#include <Arduino.h>
#include <micro_ros_platformio.h>

//ROS includes
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>

/*
  MPU6050 Raw

  A code for obtaining raw data from the MPU6050 module with the option to
  modify the data output format.

  Find the full MPU6050 library documentation here:
  https://github.com/ElectronicCats/mpu6050/wiki
*/
#include "I2Cdev.h"
#include "MPU6050.h"

#define LED_PIN 13
#define OUTPUT_READABLE_ACCELGYRO

#if defined(ARDUINO_TEENSY41)
void get_teensy_mac(uint8_t *mac) {

    //blink twice
    for(uint8_t i = 0; i < 4; i++) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(100);
    }

    for(uint8_t by=0; by<2; by++) mac[by]=(HW_OCOTP_MAC1 >> ((1-by)*8)) & 0xFF;
    for(uint8_t by=0; by<4; by++) mac[by+2]=(HW_OCOTP_MAC0 >> ((3-by)*8)) & 0xFF;
}
#endif

rcl_publisher_t publisher;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

MPU6050 mpu; //IMU object
std_msgs__msg__Int32MultiArray msg; //IMU data message
static int32_t msg_data[12]; //IMU data array
//IMU measurements + offsets
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset; 

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    
    for(int i = 0; i < 12; i++) {
      msg.data.data[i] = msg.data.data[i] + 12;
    }
    //take IMU measurement */
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    /* Subtract the offsets to get values relative to the resting position (0,0,0 when stationary) */
    ax -= ax_offset;
    ay -= ay_offset;
    az -= az_offset;
    gx -= gx_offset;
    gy -= gy_offset;
    gz -= gz_offset;

    msg.data.data[0] = ax;
    msg.data.data[1] = ay;
    msg.data.data[2] = az;
    msg.data.data[3] = gx;
    msg.data.data[4] = gy;
    msg.data.data[5] = gz;
  }
}

void setup() {
  /*--Start I2C interface-- */
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin(); 
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(38400); //Initializate Serial wo work well at 8MHz/16MHz

  /*Initialize device and check connection* */
  Serial.println("Initializing MPU...");
  mpu.initialize();
  Serial.println("Testing MPU6050 connection...");
  if(mpu.testConnection() ==  false){
    Serial.println("MPU6050 connection failed");
    while(true);
  }
  else{
    Serial.println("MPU6050 connection successful");
  }

  /* read initial offset at rest */
  mpu.getMotion6(&ax_offset, &ay_offset, &az_offset, &gx_offset, &gy_offset, &gz_offset);
  /*output */
  Serial.println("Initial offsets:");
  Serial.print("\t");
  Serial.print(ax_offset); 
  Serial.print("\t");
  Serial.print(ay_offset); 
  Serial.print("\t");
  Serial.print(az_offset); 
  Serial.print("\t");
  Serial.print(gx_offset); 
  Serial.print("\t");
  Serial.print(gy_offset); 
  Serial.print("\t");
  Serial.println(gz_offset);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  byte arduino_mac[] = { 0xAA, 0xBB, 0xCC, 0xEE, 0xDD, 0xFF };

  #if defined(ARDUINO_TEENSY41)
  get_teensy_mac(arduino_mac);
  #endif

  Serial.println("Setting transports...");

  IPAddress arduino_ip(LOCAL_IP);
  IPAddress agent_ip(AGENT_IP);
  set_microros_native_ethernet_transports(arduino_mac, arduino_ip, agent_ip, AGENT_PORT);

  delay(2000);

  allocator = rcl_get_default_allocator();

  /* blink twice
  for(uint8_t i = 0; i < 4; i++) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(100);
  } */

  delay(1000);


  Serial.println("Creating init options...");
  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  /*blink twice
  for(uint8_t i = 0; i < 4; i++) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(100);
  } */

  delay(1000);

  Serial.println("Creating ethernet node...");
  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_ethernet_node", "", &support));

  /*blink twice
  for(uint8_t i = 0; i < 4; i++) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(100);
  } */

  delay(1000);

  Serial.println("Creating publisher...");
  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    TOPIC));

  // create timer,
  const unsigned int timer_timeout = 10;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));



  //initialize IMU data array (9 measurements)
  msg.data.capacity = 6;
  msg.data.data = msg_data;
  msg.data.size = 0;

  for(int i = 0; i < 6; i++) {
    msg.data.data[i] = 0;
    msg.data.size++;
  }

}

void loop() {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
