#include <ros.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/String.h>
#include <Encoder.h>
#include <LedControl.h>
#include <Arduino.h>

// ======= MOTOR PINS =======
#define LEFT_PWM 5
#define LEFT_DIR 4
#define RIGHT_PWM 6
#define RIGHT_DIR 7

// ======= ENCODER PINS =======
#define LEFT_ENCODER_A 2
#define LEFT_ENCODER_B 3
#define RIGHT_ENCODER_A 8
#define RIGHT_ENCODER_B 9

// ======= LED MATRIX =======
#define LED_DIN 10
#define LED_CLK 11
#define LED_CS 12
#define NUM_MATRICES 4
LedControl lc = LedControl(LED_DIN, LED_CLK, LED_CS, NUM_MATRICES);

// ======= ROBOT PARAMETERS =======
const float WHEEL_RADIUS = 0.05; // meters
const float WHEEL_SEPARATION = 0.20; // meters
const float TICKS_PER_REV = 360; // encoder ticks per wheel revolution

// ======= ENCODERS =======
Encoder leftEnc(LEFT_ENCODER_A, LEFT_ENCODER_B);
Encoder rightEnc(RIGHT_ENCODER_A, RIGHT_ENCODER_B);
long lastLeftCount = 0;
long lastRightCount = 0;

// ======= ODOMETRY =======
float x = 0.0, y = 0.0, theta = 0.0;

// ======= ROS =======
ros::NodeHandle nh;
geometry_msgs::msg::Twist cmd_vel_msg;
nav_msgs::msg::Odometry odom_msg;

// ======= LED TEXT =======
String ledText = "";
unsigned long scrollInterval = 200; // default
unsigned long lastScrollTime = 0;
int scrollPos = 0;

// BLINKING EYE
bool eyeOn = false;
unsigned long lastBlinkTime = 0;
const int blinkInterval = 500;

// ======= FONT =======
// 5x7 font for characters 32-127 (basic ASCII)
const uint8_t font[96][5] = {
  {0x00,0x00,0x00,0x00,0x00}, // ' ' (space)
  {0x00,0x00,0x5F,0x00,0x00}, // '!'
  {0x00,0x07,0x00,0x07,0x00}, // '"'
  // ... add remaining characters as needed
};

// ======= UTILITY =======
void drawEye(bool state){
  int mat = 3; // last matrix
  lc.clearDisplay(mat);
  if(state){
    lc.setLed(mat,3,5,true);
    lc.setLed(mat,4,5,true);
  }
}

// Draw scrolling text across first 3 matrices (24 columns)
void drawScrollingText(){
  // Clear first 3 matrices
  for(int m=0;m<3;m++) for(int row=0;row<8;row++) lc.setColumn(m,row,0x00);

  int len = ledText.length();
  int colOffset = 0;

  for(int c=0; c<len; c++){
    char ch = ledText[c];
    if(ch < 32 || ch > 127) ch = 32;
    for(int col=0; col<5; col++){
      int globalCol = scrollPos + colOffset;
      if(globalCol >= 0 && globalCol < 24){
        int mat = globalCol / 8;
        int colInMat = globalCol % 8;
        for(int row=0; row<7; row++){
          bool pixel = font[ch-32][col] & (1 << row);
          lc.setLed(mat,row,colInMat,pixel);
        }
      }
      colOffset++;
    }
    colOffset++; // 1 column space between letters
  }
}

// ======= CALLBACKS =======
void cmdVelCallback(const geometry_msgs::msg::Twist& msg){
  float v = msg.linear.x;
  float w = msg.angular.z;

  float left_speed  = v - w*WHEEL_SEPARATION/2.0;
  float right_speed = v + w*WHEEL_SEPARATION/2.0;

  int leftPWM  = constrain(int(left_speed*255),-255,255);
  int rightPWM = constrain(int(right_speed*255),-255,255);

  if(leftPWM>=0){digitalWrite(LEFT_DIR,HIGH); analogWrite(LEFT_PWM,leftPWM);}
  else {digitalWrite(LEFT_DIR,LOW); analogWrite(LEFT_PWM,-leftPWM);}
  if(rightPWM>=0){digitalWrite(RIGHT_DIR,HIGH); analogWrite(RIGHT_PWM,rightPWM);}
  else {digitalWrite(RIGHT_DIR,LOW); analogWrite(RIGHT_PWM,-rightPWM);}
}

// ROS 2 LED text: "text:interval"
void ledTextCallback(const std_msgs::String& msg){
  String data = String(msg.data.c_str());
  int sep = data.indexOf(':');
  if(sep != -1){
    ledText = data.substring(0,sep);
    scrollInterval = data.substring(sep+1).toInt();
  } else {
    ledText = data;
    scrollInterval = 200;
  }
  scrollPos = 24; // start outside right edge
}

// ======= ROS =======
ros::Subscriber<geometry_msgs::msg::Twist> cmd_vel_sub("cmd_vel",&cmdVelCallback);
ros::Subscriber<std_msgs::String> led_sub("led_text",&ledTextCallback);
ros::Publisher odom_pub("odom",&odom_msg);

// ======= SETUP =======
void setup(){
  pinMode(LEFT_PWM,OUTPUT); pinMode(LEFT_DIR,OUTPUT);
  pinMode(RIGHT_PWM,OUTPUT); pinMode(RIGHT_DIR,OUTPUT);

  for(int i=0;i<NUM_MATRICES;i++){
    lc.shutdown(i,false);
    lc.setIntensity(i,8);
    lc.clearDisplay(i);
  }

  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(led_sub);
  nh.advertise(odom_pub);

  odom_msg.header.frame_id="odom";
  odom_msg.child_frame_id="base_link";
}

// ======= LOOP =======
unsigned long lastOdomTime=0;
void loop(){
  nh.spinOnce();
  unsigned long now=millis();
  float dt=(now-lastOdomTime)/1000.0;

  // ===== ODOMETRY =====
  if(dt>=0.05){
    lastOdomTime=now;
    long l=leftEnc.read(), r=rightEnc.read();
    long dl=l-lastLeftCount, dr=r-lastRightCount;
    lastLeftCount=l; lastRightCount=r;
    float ld=2*3.14159*WHEEL_RADIUS*dl/TICKS_PER_REV;
    float rd=2*3.14159*WHEEL_RADIUS*dr/TICKS_PER_REV;
    float dist=(ld+rd)/2.0, dtheta=(rd-ld)/WHEEL_SEPARATION;
    x+=dist*cos(theta+dtheta/2.0);
    y+=dist*sin(theta+dtheta/2.0);
    theta+=dtheta;
    odom_msg.header.stamp=nh.now();
    odom_msg.pose.pose.position.x=x;
    odom_msg.pose.pose.position.y=y;
    odom_msg.pose.pose.position.z=0;
    odom_msg.pose.pose.orientation.z=sin(theta/2.0);
    odom_msg.pose.pose.orientation.w=cos(theta/2.0);
    odom_msg.twist.twist.linear.x=dist/dt;
    odom_msg.twist.twist.angular.z=dtheta/dt;
    odom_pub.publish(&odom_msg);
  }

  // ===== LED DISPLAY =====
  if(ledText.length()>0){
    if(now-lastScrollTime>=scrollInterval){
      scrollPos--;
      lastScrollTime=now;
      if(scrollPos<-int(ledText.length()*6)) scrollPos=24; // loop
    }
    drawScrollingText();
  } else {
    if(now-lastBlinkTime>=blinkInterval){
      eyeOn=!eyeOn; lastBlinkTime=now;
    }
    drawEye(eyeOn);
  }
}
