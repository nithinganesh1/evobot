#include <ros.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <Encoder.h>

// MOTOR PINS
#define LEFT_PWM 5
#define LEFT_DIR 4
#define RIGHT_PWM 6
#define RIGHT_DIR 7

// ENCODER PINS
#define LEFT_ENCODER_A 2
#define LEFT_ENCODER_B 3
#define RIGHT_ENCODER_A 8
#define RIGHT_ENCODER_B 9

// ROBOT PARAMETERS
const float WHEEL_RADIUS = 0.05;   // meters
const float WHEEL_SEPARATION = 0.20; // meters
const float TICKS_PER_REV = 360; // OE775 encoder ticks per wheel revolution

// ENCODER OBJECTS
Encoder leftEnc(LEFT_ENCODER_A, LEFT_ENCODER_B);
Encoder rightEnc(RIGHT_ENCODER_A, RIGHT_ENCODER_B);
long lastLeftCount = 0;
long lastRightCount = 0;

// ODOMETRY
float x = 0.0, y = 0.0, theta = 0.0;

// ROS 2 NodeHandle
ros::NodeHandle nh;

// Messages
geometry_msgs::msg::Twist cmd_vel_msg;
nav_msgs::msg::Odometry odom_msg;

// ===== CMD_VEL CALLBACK =====
void cmdVelCallback(const geometry_msgs::msg::Twist& msg){
  float v = msg.linear.x;
  float w = msg.angular.z;

  float left_speed  = v - w * WHEEL_SEPARATION / 2.0;
  float right_speed = v + w * WHEEL_SEPARATION / 2.0;

  int leftPWM  = constrain(int(left_speed * 255), -255, 255);
  int rightPWM = constrain(int(right_speed * 255), -255, 255);

  if(leftPWM >= 0){ digitalWrite(LEFT_DIR, HIGH); analogWrite(LEFT_PWM, leftPWM); }
  else { digitalWrite(LEFT_DIR, LOW); analogWrite(LEFT_PWM, -leftPWM); }

  if(rightPWM >= 0){ digitalWrite(RIGHT_DIR, HIGH); analogWrite(RIGHT_PWM, rightPWM); }
  else { digitalWrite(RIGHT_DIR, LOW); analogWrite(RIGHT_PWM, -rightPWM); }
}

// Subscriber and Publisher
ros::Subscriber<geometry_msgs::msg::Twist> cmd_vel_sub("cmd_vel", &cmdVelCallback);
ros::Publisher odom_pub("odom", &odom_msg);

// ===== SETUP =====
void setup(){
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);

  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.advertise(odom_pub);

  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";
}

// ===== LOOP =====
unsigned long last_time = 0;
void loop(){
  nh.spinOnce();
  unsigned long now = millis();
  float dt = (now - last_time)/1000.0;
  if(dt < 0.05) return; // 20 Hz update
  last_time = now;

  // Read encoder ticks
  long leftCount  = leftEnc.read();
  long rightCount = rightEnc.read();

  long deltaLeft  = leftCount - lastLeftCount;
  long deltaRight = rightCount - lastRightCount;
  lastLeftCount = leftCount;
  lastRightCount = rightCount;

  // Convert to distance
  float leftDist  = 2.0 * 3.14159 * WHEEL_RADIUS * deltaLeft / TICKS_PER_REV;
  float rightDist = 2.0 * 3.14159 * WHEEL_RADIUS * deltaRight / TICKS_PER_REV;

  float dist = (leftDist + rightDist)/2.0;
  float dtheta = (rightDist - leftDist)/WHEEL_SEPARATION;

  x += dist * cos(theta + dtheta/2.0);
  y += dist * sin(theta + dtheta/2.0);
  theta += dtheta;

  // Fill odometry message
  odom_msg.header.stamp = nh.now();
  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.z = 0;
  odom_msg.pose.pose.orientation.z = sin(theta/2.0);
  odom_msg.pose.pose.orientation.w = cos(theta/2.0);

  odom_msg.twist.twist.linear.x = dist/dt;
  odom_msg.twist.twist.angular.z = dtheta/dt;

  odom_pub.publish(&odom_msg);
}
