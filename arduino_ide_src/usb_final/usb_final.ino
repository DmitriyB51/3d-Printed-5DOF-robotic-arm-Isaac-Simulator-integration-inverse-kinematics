#include <ESP32Servo.h>
#define MICRO_ROS_TRANSPORT_SERIAL
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>


#define SERVO1_PIN    13
#define SERVO2_PIN    14
#define SERVO3_PIN    15
#define SERVO4_PIN    12

#define SERVO5_PIN    19



#define SERVO_MIN_US  500
#define SERVO_MAX_US  2500

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

Servo servo5;


// micro-ROS 
rcl_subscription_t   subscriber;
rclc_executor_t      executor;
rcl_node_t           node;
rclc_support_t       support;
rcl_allocator_t      allocator;
std_msgs__msg__Float32MultiArray servo_msg;


#define RCCHECK(fn) { \
  rcl_ret_t rc = fn; \
  if (rc != RCL_RET_OK) { \
    Serial.print("Error in " #fn ": "); \
    Serial.println(rc); \
    while(1) { delay(100); } \
  } \
}


void servo_callback(const void * msgin)
{
  const std_msgs__msg__Float32MultiArray * msg =
    (const std_msgs__msg__Float32MultiArray *)msgin;

  if (msg->data.size < 5) {
    Serial.println("[WARN] not enough data in servo_angles");
    return;
  }

  float angle1 = msg->data.data[0];
  float angle2 = msg->data.data[1];
  float angle3 = msg->data.data[2];
  float angle4 = msg->data.data[3];

  float angle5 = msg->data.data[4];



  if (angle1 < 0) angle1 = 0;
  if (angle1 > 180) angle1 = 180;

  if (angle2 < 0) angle2 = 0;
  if (angle2 > 180) angle2 = 180;

  if (angle3 < 0) angle3 = 0;
  if (angle3 > 180) angle3 = 180;

  if (angle4 < 0) angle4 = 0;
  if (angle4 > 180) angle4 = 180;

  if (angle5 < 0) angle5 = 0;
  if (angle5 > 180) angle5 = 180;



  int pulse1 = SERVO_MIN_US + (angle1 / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);
  int pulse2 = SERVO_MIN_US + (angle2 / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);
  int pulse3 = SERVO_MIN_US + (angle3 / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);
  int pulse4 = SERVO_MIN_US + (angle4 / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);

  int pulse5 = SERVO_MIN_US + (angle5 / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);

  servo1.writeMicroseconds(pulse1);
  servo2.writeMicroseconds(pulse2);
  servo3.writeMicroseconds(pulse3);
  servo4.writeMicroseconds(pulse4);

  servo5.writeMicroseconds(pulse5);

  Serial.printf(
    "[CB] angles: %.2f %.2f %.2f %.2f %.2f | pulses: %d %d %d %d %d\n",
    angle1, angle2, angle3, angle4, angle5,
    pulse1, pulse2, pulse3, pulse4, pulse5
  );
}

//setup 
void setup()
{
  Serial.begin(115200);
  delay(1000);

  
  servo1.attach(SERVO1_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servo2.attach(SERVO2_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servo3.attach(SERVO3_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servo4.attach(SERVO4_PIN, SERVO_MIN_US, SERVO_MAX_US);

  servo5.attach(SERVO5_PIN, SERVO_MIN_US, SERVO_MAX_US);



  // usb
  set_microros_transports(); 


  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(
    &node,
    "esp32_servo_node",
    "",
    &support
  ));

  //  memory
  std_msgs__msg__Float32MultiArray__init(&servo_msg);
  servo_msg.data.data = (float*) malloc(5 * sizeof(float));
  servo_msg.data.capacity = 5;
  servo_msg.data.size = 0;

  //creating subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "servo_angles"
  ));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &servo_msg,
    &servo_callback,
    ON_NEW_DATA
  ));

  Serial.println("micro-ROS serial node ready");
}


void loop()
{
  rclc_executor_spin_some(&executor, 0);
}

