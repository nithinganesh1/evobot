#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// MD20 pins (change according to your wiring)
#define LEFT_PWM 25
#define LEFT_DIR 14
#define RIGHT_PWM 26
#define RIGHT_DIR 33

#define LED_PIN 24

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1){digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(100);}} }

unsigned long last_msg_time = 0;
bool has_msg = false;

void stop_motors() {
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);
  digitalWrite(LED_PIN, LOW);
}

void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * twist = (const geometry_msgs__msg__Twist *)msgin;

  float linear = twist->linear.x;
  float angular = twist->angular.z;

  // Deadzone: small values considered zero
  if (abs(linear) < 0.05 && abs(angular) < 0.05) {
    stop_motors();
    has_msg = false;
    return;
  }

  has_msg = true;
  last_msg_time = millis();

  // Differential drive
  float left_speed  = linear - angular;
  float right_speed = linear + angular;

  // Clip to [-1, 1]
  left_speed  = max(-1.0f, min(1.0f, left_speed));
  right_speed = max(-1.0f, min(1.0f, right_speed));

  // Set directions
  digitalWrite(LEFT_DIR,  left_speed >= 0 ? HIGH : LOW);
  digitalWrite(RIGHT_DIR, right_speed >= 0 ? HIGH : LOW);

  // Set speed
  analogWrite(LEFT_PWM,  abs(left_speed) * 255);
  analogWrite(RIGHT_PWM, abs(right_speed) * 255);

  // LED ON when moving
  digitalWrite(LED_PIN, HIGH);
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);
  stop_motors();

  Serial.begin(115200);
  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_md20_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"
  ));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));

  // Stop motors if no message received in last 500ms
  if (has_msg && (millis() - last_msg_time > 50)) {
    stop_motors();
    has_msg = false;
  }
}
