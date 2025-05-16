#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>


rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;


#define LED_PIN 2

int motor1Pin1 = 16; 
int motor1Pin2 = 17; 
int enable1Pin = 4; 

int motor2Pin1 = 5; 
int motor2Pin2 = 18; 
int enable2Pin = 19; 

const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;




#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

//twist message cb
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  
  float V = msg->linear.x;
  float W = msg->angular.z;
  float radius = 0.03;
  float L = 0.13;
  
  float left_speed = (V - W * L / 2.0) / radius;
  float left_speed_control = abs(left_speed) * 5;
  left_speed_control = constrain(left_speed_control, 0, 255);
  
  
  float right_speed = (V + W * L / 2.0) / radius;
  float right_speed_control = abs(right_speed) * 5;
  right_speed_control = constrain(right_speed_control, 0, 255);
  

  if(left_speed > 0){
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor1Pin1, HIGH);
    ledcWrite(enable1Pin, left_speed_control);
  }
  else{
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor1Pin1, LOW);
    ledcWrite(enable1Pin, left_speed_control);
  }

  if(right_speed > 0){
    digitalWrite(motor2Pin2, LOW);
    digitalWrite(motor2Pin1, HIGH);
    ledcWrite(enable2Pin, right_speed_control);
  }
  else{
    digitalWrite(motor2Pin2, HIGH);
    digitalWrite(motor2Pin1, LOW);
    ledcWrite(enable2Pin, right_speed_control);
  }

  
}

void setup() {
  set_microros_transports();
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);

  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);

  pinMode(LED_PIN, OUTPUT);

  ledcAttachChannel(enable1Pin, freq, resolution, pwmChannel);
  ledcAttachChannel(enable2Pin, freq, resolution, 1);  
  set_microros_wifi_transports("MANH", "manh12345", "10.42.0.1", 8888);

  //MANH, manh12345 ,192.168.196.230 // P309,   192.168.0.103

  digitalWrite(LED_PIN, HIGH);
  delay(2000);

  allocator = rcl_get_default_allocator();

   //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "micro_ros_arduino_twist_subscriber"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

}

void loop() {
  delay(50);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50)));
}