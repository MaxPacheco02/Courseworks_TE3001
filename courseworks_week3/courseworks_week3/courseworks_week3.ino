// Max Pacheco Ramirez

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

#define LED1 12
#define LED2 13
#define POT 15

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

rcl_subscription_t pwm_sub_;
rcl_publisher_t raw_pot_pub_, voltage_pub_;

std_msgs__msg__Float32 voltage_msg, raw_msg, pwm_msg;

rclc_executor_t exec_1, exec_2, exec_pwm;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer_1, timer_2;

int pot = 0;
float voltage = 0;
int duty_cycle = 0;

void error_loop(){
  while(1){
    digitalWrite(LED1, !digitalRead(LED1));
    delay(100);
  }
}

void timer_callback_1(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    pot = analogRead(POT);
    voltage = (float(pot)/4095.0)*3.3;
  }
}

void timer_callback_2(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&raw_pot_pub_, &raw_msg, NULL));
    raw_msg.data = pot;
    RCSOFTCHECK(rcl_publish(&voltage_pub_, &voltage_msg, NULL));
    voltage_msg.data = voltage;
  }
}

void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  if(msg->data <= 0){
    duty_cycle = 0;
  }else if(msg->data >= 100){
    duty_cycle = 100;
  }else{
    duty_cycle = msg->data;
  }
  analogWrite(LED1, (float(duty_cycle)/100.0) * 255);  
  analogWrite(LED2, (float(duty_cycle)/100.0) * 255);  
}

void setup() {
  set_microros_transports();
  
  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, HIGH);  
  pinMode(LED2, OUTPUT);
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &pwm_sub_,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/micro_ros_esp32/pwm_duty_cycle"));

  // create publisher raw
  RCCHECK(rclc_publisher_init_default(
    &raw_pot_pub_,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/micro_ros_esp32/raw_pot"));

  // create publisher volt
  RCCHECK(rclc_publisher_init_default(
    &voltage_pub_,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/micro_ros_esp32/voltage"));

  // create timer 10 ms
  const unsigned int timer_timeout_10 = 10;
  RCCHECK(rclc_timer_init_default(
    &timer_1,
    &support,
    RCL_MS_TO_NS(timer_timeout_10),
    timer_callback_1));

  // create timer 100 ms
    const unsigned int timer_timeout_100 = 100;
  RCCHECK(rclc_timer_init_default(
    &timer_2,
    &support,
    RCL_MS_TO_NS(timer_timeout_100),
    timer_callback_2));

  // create executor
  RCCHECK(rclc_executor_init(&exec_pwm, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&exec_1, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&exec_2, &support.context, 1, &allocator));

  RCCHECK(rclc_executor_add_timer(&exec_1, &timer_1));
  RCCHECK(rclc_executor_add_timer(&exec_2, &timer_2));
  RCCHECK(rclc_executor_add_subscription(&exec_pwm, &pwm_sub_, &pwm_msg, &subscription_callback, ON_NEW_DATA));

  raw_msg.data = 0;
  voltage_msg.data = 0;
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&exec_1, RCL_MS_TO_NS(100)));
  RCSOFTCHECK(rclc_executor_spin_some(&exec_2, RCL_MS_TO_NS(100)));
  RCSOFTCHECK(rclc_executor_spin_some(&exec_pwm, RCL_MS_TO_NS(100)));
}