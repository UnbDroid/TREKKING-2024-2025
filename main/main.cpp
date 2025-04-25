// #include "MotorDC.h"
// #include "PS4BT.h"
// #include "PinConfig.h"
// #include "RobotProperties.h"
// #include "EspRaspRobot.h"
// #include "RobotPs4Controller.h"
// #include "btd_vhci.h"
// #include "esp_attr.h"
// #include "esp_log.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/idf_additions.h"
// #include "freertos/projdefs.h"
// #include "freertos/task.h"
// #include "nvs_flash.h"

// MotorDC left_front_motor(ENCA_LEFT_FRONT, ENCB_LEFT_FRONT, L_PWM_LEFT_FRONT,
//                          R_PWM_LEFT_FRONT, LEDC_CHANNEL_LEFT_FRONT_L_PWM,
//                          LEDC_CHANNEL_LEFT_FRONT_R_PWM);
// MotorDC left_back_motor(ENCA_LEFT_BACK, ENCB_LEFT_BACK, L_PWM_LEFT_BACK,
//                         R_PWM_LEFT_BACK, LEDC_CHANNEL_LEFT_BACK_L_PWM,
//                         LEDC_CHANNEL_LEFT_BACK_R_PWM);
// MotorDC right_front_motor(ENCA_RIGHT_FRONT, ENCB_RIGHT_FRONT, L_PWM_RIGHT_FRONT,
//                           R_PWM_RIGHT_FRONT, LEDC_CHANNEL_RIGHT_FRONT_L_PWM,
//                           LEDC_CHANNEL_RIGHT_FRONT_R_PWM);
// MotorDC right_back_motor(ENCA_RIGHT_BACK, ENCB_RIGHT_BACK, L_PWM_RIGHT_BACK,
//                          R_PWM_RIGHT_BACK, LEDC_CHANNEL_RIGHT_BACK_L_PWM,
//                          LEDC_CHANNEL_RIGHT_BACK_R_PWM);

// void IRAM_ATTR read_encoder_left_front(void *arg) {
//   left_front_motor.read_encoder(arg);
// }

// void IRAM_ATTR read_encoder_left_back(void *arg) {
//   left_back_motor.read_encoder(arg);
// }

// void IRAM_ATTR read_encoder_right_front(void *arg) {
//   right_front_motor.read_encoder(arg);
// }
// // ta dando problema nisso aqui
// void IRAM_ATTR read_encoder_right_back(void *arg) {
//   right_back_motor.read_encoder(arg);
// }

// void robot_setup() {
  
//   pin_configuration();
  
//   gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
//   gpio_isr_handler_add((gpio_num_t)ENCA_LEFT_FRONT, read_encoder_left_front,
//   (void *)ENCA_LEFT_FRONT);
//   gpio_isr_handler_add((gpio_num_t)ENCA_LEFT_BACK, read_encoder_left_back,
//   (void *)ENCA_LEFT_BACK);
//   gpio_isr_handler_add((gpio_num_t)ENCA_RIGHT_FRONT, read_encoder_right_front,
//   (void *)ENCA_RIGHT_FRONT);
//   gpio_isr_handler_add((gpio_num_t)ENCA_RIGHT_BACK, read_encoder_right_back,
//   (void *)ENCA_RIGHT_BACK);
  
//   left_front_motor.configure_motor(300, 1.4, 1.2, 0.00001);
//   left_back_motor.configure_motor(300, 1.8, 0.5, 0);
//   right_front_motor.configure_motor(300, 1.3, 0.3, 0);
//   right_back_motor.configure_motor(300, 1.3, 0.3, 0);
  
  
// }

// // PS4BT PS4;
// // RobotPs4Controller robo(&right_front_motor, &right_back_motor,
// //   &left_front_motor, &left_back_motor);
//   RobotProperties robotProperties(&right_front_motor, &right_back_motor,
//     &left_front_motor, &left_back_motor);
//   EspRaspRobot autonomousRobot(
//     &right_front_motor, &right_back_motor, &left_front_motor, &left_back_motor,
//     &robotProperties); // Pass the RobotProperties object to the constructor
// // void task_controll(void *task_params) { robo.task_robot_controll(task_params); }
// void task_velocity(void *task_params) {
//   while (1) {
//     left_back_motor.fetch_rpm();
//     left_front_motor.fetch_rpm();
//     right_back_motor.fetch_rpm();
//     right_front_motor.fetch_rpm();
//     left_front_motor.move_pid(80);
//     left_back_motor.move_pid(80);
//     right_front_motor.move_pid(80);
//     right_back_motor.move_pid(80);
//     // ESP_LOGI("v", "%f %f %f %f", left_front_motor.current_speed_rpm,
//     //          left_back_motor.current_speed_rpm,
//     //          right_front_motor.current_speed_rpm,
//     //          right_back_motor.current_speed_rpm);
//     vTaskDelay(pdMS_TO_TICKS(10));
//   }
// }

// void rosStuff(void *task_params) {
//   autonomousRobot.micro_ros_run();
// }

// extern "C" void app_main(void) {
  
//   esp_err_t ret;
//   robot_setup();
//   autonomousRobot.micro_ros_setup();
  
//   // Initialize the PS4 controller -->
  
//   // ret = nvs_flash_init();
//   // ret = btd_vhci_init();
//   // btd_vhci_autoconnect(&PS4);
//   // robo.set_controller(&PS4);
//   // xTaskCreatePinnedToCore(task_controll, "ps4_loop_task", 10 * 1024, NULL, 2, NULL);
//   // xTaskCreatePinnedToCore(task_velocity, "velocity", 10 * 1024, NULL, 2, NULL);

//   // <-- Until here

//   // Set all motors to go forward at 120

//   xTaskCreatePinnedToCore(task_velocity, "velocity", 1 * 1024, NULL, 1, NULL, 0);
//   // xTaskCreatePinnedToCore(rosStuff, "ros", 2 * 1024, NULL, 2, NULL, 1);
  

// }


// Version with just the subscription

#include "MotorDC.h"
#include "PS4BT.h"
#include "PinConfig.h"
#include "RobotProperties.h"
#include "EspRaspRobot.h"
#include "RobotPs4Controller.h"
#include "btd_vhci.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#define RCCHECK(fn)                                                                \
  {                                                                                \
    rcl_ret_t temp_rc = fn;                                                        \
    if ((temp_rc != RCL_RET_OK))                                                   \
    {                                                                              \
      printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
      vTaskDelete(NULL);                                                           \
    }                                                                              \
  }
#define RCSOFTCHECK(fn)                                                              \
  {                                                                                  \
    rcl_ret_t temp_rc = fn;                                                          \
    if ((temp_rc != RCL_RET_OK))                                                     \
    {                                                                                \
      printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
    }                                                                                \
  }

  uart_port_t uart_port = UART_NUM_0;

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

MotorDC left_front_motor(ENCA_LEFT_FRONT, ENCB_LEFT_FRONT, L_PWM_LEFT_FRONT,
                         R_PWM_LEFT_FRONT, LEDC_CHANNEL_LEFT_FRONT_L_PWM,
                         LEDC_CHANNEL_LEFT_FRONT_R_PWM);
MotorDC left_back_motor(ENCA_LEFT_BACK, ENCB_LEFT_BACK, L_PWM_LEFT_BACK,
                        R_PWM_LEFT_BACK, LEDC_CHANNEL_LEFT_BACK_L_PWM,
                        LEDC_CHANNEL_LEFT_BACK_R_PWM);
MotorDC right_front_motor(ENCA_RIGHT_FRONT, ENCB_RIGHT_FRONT, L_PWM_RIGHT_FRONT,
                          R_PWM_RIGHT_FRONT, LEDC_CHANNEL_RIGHT_FRONT_L_PWM,
                          LEDC_CHANNEL_RIGHT_FRONT_R_PWM);
MotorDC right_back_motor(ENCA_RIGHT_BACK, ENCB_RIGHT_BACK, L_PWM_RIGHT_BACK,
                         R_PWM_RIGHT_BACK, LEDC_CHANNEL_RIGHT_BACK_L_PWM,
                         LEDC_CHANNEL_RIGHT_BACK_R_PWM);

void subscription_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *incoming_msg = (const geometry_msgs__msg__Twist *)msgin;
  msg = *incoming_msg;
}

void IRAM_ATTR read_encoder_left_front(void *arg)
{
  left_front_motor.read_encoder(arg);
}

void IRAM_ATTR read_encoder_left_back(void *arg)
{
  left_back_motor.read_encoder(arg);
}

void IRAM_ATTR read_encoder_right_front(void *arg)
{
  right_front_motor.read_encoder(arg);
}
// ta dando problema nisso aqui
void IRAM_ATTR read_encoder_right_back(void *arg)
{
  right_back_motor.read_encoder(arg);
}

void robot_setup()
{
  
  pin_configuration();
  
  gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
  gpio_isr_handler_add((gpio_num_t)ENCA_LEFT_FRONT, read_encoder_left_front,
  (void *)ENCA_LEFT_FRONT);
  gpio_isr_handler_add((gpio_num_t)ENCA_LEFT_BACK, read_encoder_left_back,
  (void *)ENCA_LEFT_BACK);
  gpio_isr_handler_add((gpio_num_t)ENCA_RIGHT_FRONT, read_encoder_right_front,
  (void *)ENCA_RIGHT_FRONT);
  gpio_isr_handler_add((gpio_num_t)ENCA_RIGHT_BACK, read_encoder_right_back,
  (void *)ENCA_RIGHT_BACK);
  
  left_front_motor.configure_motor(300, 1.4, 1.2, 0.00001);
  left_back_motor.configure_motor(300, 1.8, 0.5, 0);
  right_front_motor.configure_motor(300, 1.3, 0.3, 0);
  right_back_motor.configure_motor(300, 1.3, 0.3, 0);
}

// RobotProperties robotProperties;

// PS4BT PS4;
// RobotPs4Controller robo(&right_front_motor, &right_back_motor,
//   &left_front_motor, &left_back_motor);

// EspRaspRobot robot(&left_front_motor, &right_front_motor,
//                    &left_back_motor, &right_back_motor, &robotProperties);

void *task_params = NULL;

// void RPM_fetching(void *task_params)
// {
//   while (1)
//   {
//     left_back_motor.fetch_rpm();
//     left_front_motor.fetch_rpm();
//     right_back_motor.fetch_rpm();
//     right_front_motor.fetch_rpm();
//     vTaskDelay(pdMS_TO_TICKS(10));
//   }
// }

void task_velocity() {
  // Extract linear and angular velocities from the message
  double linear_x = (double)msg.linear.x;  // Linear velocity in m/s
  double angular_z = (double)msg.angular.z; // Angular velocity in rad/s

  // Multiply both by 5

  linear_x = linear_x * 2;
  angular_z = angular_z * 3;
  
  // Convert velocities to motor speeds using differential drive kinematics
  double wheel_base = 0.325; // Distance between wheels (meters)
  double wheel_radius = WHEEL_RADIUS_METERS; // Radius of the wheels (meters)
  
  // Compute individual wheel speeds in RPS
  double left_speed_mps = linear_x - (angular_z * wheel_base / 2);
  double right_speed_mps = linear_x + (angular_z * wheel_base / 2);
  
  // Convert wheel speeds from RPS to RPM
  double left_speed_rpm = (left_speed_mps / (2 * M_PI * wheel_radius)) * 60.0;
  double right_speed_rpm = (right_speed_mps / (2 * M_PI * wheel_radius)) * 60.0;

  // Fetch current RPM values from the motors
  left_front_motor.fetch_rpm();
  left_back_motor.fetch_rpm();
  right_front_motor.fetch_rpm();
  right_back_motor.fetch_rpm();

  // Limit speed to a maximum value (50 RPM in this case)
  if (left_speed_rpm > 40) {
    left_speed_rpm = 40;
  }
  if (right_speed_rpm > 40) {
    right_speed_rpm = 40;
  }
  if (left_speed_rpm < -40) {
    left_speed_rpm = -40;
  }
  if (right_speed_rpm < -40) {
    right_speed_rpm = -40;
  }
  
  // Send the desired RPM values to the motors using PID control
  left_front_motor.move_pid(left_speed_rpm);
  left_back_motor.move_pid(left_speed_rpm);
  right_front_motor.move_pid(right_speed_rpm);
  right_back_motor.move_pid(right_speed_rpm);
  
  // Delay to allow the FreeRTOS task to yield
  vTaskDelay(pdMS_TO_TICKS(10));
}

void rosStuff(void *task_params) {
  rclc_executor_t executor;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  
  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // create node
  rcl_node_t node;
  RCCHECK(rclc_node_init_default(&node, "int32_subscriber_rclc", "", &support));
  
  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel_nav"));
    
  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  
  while(1){
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10000));
    task_velocity();
    usleep(10000);
  }

}

extern "C" void app_main(void)
{
  
  #if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
		rmw_uros_set_custom_transport(
      true,
      (void *) &uart_port,
		esp32_serial_open,
		esp32_serial_close,
		esp32_serial_write,
		esp32_serial_read
  );
	#else
	#error micro-ROS transports misconfigured
	#endif  // RMW_UXRCE_TRANSPORT_CUSTOM
  
  robot_setup();
    

  // xTaskCreate(RPM_fetching, "RPM_fetching", 4 * 1024, NULL, 1, NULL);
  xTaskCreate(rosStuff, "task-ros", 4 * 1024, NULL, 1, NULL);
  // xTaskCreate(task_velocity, "task_velocity", 4 * 1024, NULL, 1, NULL);
  
  // right_front_motor.set_direction_pwm(1, 120);
  // right_back_motor.set_direction_pwm(1, 120);

  // free resources

  vTaskDelete(NULL);
}
