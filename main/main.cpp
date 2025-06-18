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
// MotorDC right_front_motor(ENCA_RIGHT_FRONT, ENCB_RIGHT_FRONT,
// L_PWM_RIGHT_FRONT,
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
//   gpio_isr_handler_add((gpio_num_t)ENCA_RIGHT_FRONT,
//   read_encoder_right_front, (void *)ENCA_RIGHT_FRONT);
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
//     &right_front_motor, &right_back_motor, &left_front_motor,
//     &left_back_motor, &robotProperties); // Pass the RobotProperties object
//     to the constructor
// // void task_controll(void *task_params) {
// robo.task_robot_controll(task_params); } void task_velocity(void
// *task_params) {
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
//   // xTaskCreatePinnedToCore(task_controll, "ps4_loop_task", 10 * 1024, NULL,
//   2, NULL);
//   // xTaskCreatePinnedToCore(task_velocity, "velocity", 10 * 1024, NULL, 2,
//   NULL);

//   // <-- Until here

//   // Set all motors to go forward at 120

//   xTaskCreatePinnedToCore(task_velocity, "velocity", 1 * 1024, NULL, 1, NULL,
//   0);
//   // xTaskCreatePinnedToCore(rosStuff, "ros", 2 * 1024, NULL, 2, NULL, 1);

// }

// Version with just the subscription

#include "EspRaspRobot.h"
#include "MotorDC.h"
#include "PS4BT.h"
#include "PinConfig.h"
#include "RobotProperties.h"
#include "RobotPs4Controller.h"
#include "btd_vhci.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "geometry_msgs/msg/vector3.h"
#include "math.h"
#include "nvs_flash.h"
#include <cinttypes>
#include <iterator>

#define RCCHECK(fn)                                                            \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      printf("Failed status on line %d: %d. Aborting.\n", __LINE__,            \
             (int)temp_rc);                                                    \
      vTaskDelete(NULL);                                                       \
    }                                                                          \
  }
#define RCSOFTCHECK(fn)                                                        \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      printf("Failed status on line %d: %d. Continuing.\n", __LINE__,          \
             (int)temp_rc);                                                    \
    }                                                                          \
  }

uart_port_t uart_port = UART_NUM_0;

rcl_subscription_t subscriber;
rcl_publisher_t rpm_publisher;
geometry_msgs__msg__Vector3 rpm_msg;
geometry_msgs__msg__Twist msg;

bool is_controller_on = false;

double linear_x_prev[5] = {0};
double angular_z_prev[5] = {0};

MotorDC left_front_motor(ENCA_LEFT_FRONT, PWM_LEFT_FRONT, L_IN_LEFT_FRONT,
                         R_IN_LEFT_FRONT, LEDC_CHANNEL_LEFT_FRONT_PWM);
MotorDC left_back_motor(ENCA_LEFT_BACK, PWM_LEFT_BACK, L_IN_LEFT_BACK,
                        R_IN_LEFT_BACK, LEDC_CHANNEL_LEFT_BACK_PWM);
MotorDC right_front_motor(ENCA_RIGHT_FRONT, PWM_RIGHT_FRONT, L_IN_RIGHT_FRONT,
                          R_IN_RIGHT_FRONT, LEDC_CHANNEL_RIGHT_FRONT_PWM);
MotorDC right_back_motor(ENCA_RIGHT_BACK, PWM_RIGHT_BACK, L_IN_RIGHT_BACK,
                         R_IN_RIGHT_BACK, LEDC_CHANNEL_RIGHT_BACK_PWM);

void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *incoming_msg =
      (const geometry_msgs__msg__Twist *)msgin;
  msg = *incoming_msg;
}

void IRAM_ATTR read_encoder_left_front(void *arg) {
  left_front_motor.read_encoder(arg);
}

void IRAM_ATTR read_encoder_left_back(void *arg) {
  left_back_motor.read_encoder(arg);
}

void IRAM_ATTR read_encoder_right_front(void *arg) {
  right_front_motor.read_encoder(arg);
}
// ta dando problema nisso aqui
void IRAM_ATTR read_encoder_right_back(void *arg) {
  right_back_motor.read_encoder(arg);
}

void robot_setup() {

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
  left_front_motor.configure_motor(300, 2, 0.0, 0);
  right_front_motor.configure_motor(300, 2, 0.0, 0);
  left_back_motor.configure_motor(300, 2, 0.0, 0);
  right_back_motor.configure_motor(480, 3, 0.0, 0);
}

// RobotProperties robotProperties;

PS4BT Ps4;
RobotPs4Controller robo(&right_front_motor, &right_back_motor,
                        &left_front_motor, &left_back_motor);
// EspRaspRobot robot(&left_front_motor,
//   &right_front_motor,
//                    &left_back_motor, &right_back_motor, &robotProperties);

void *task_params = NULL;

void RPM_fetching(void *task_params) {
  while (1) {
    left_back_motor.fetch_rpm();
    left_front_motor.fetch_rpm();
    right_back_motor.fetch_rpm();
    right_front_motor.fetch_rpm();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void task_velocity() {
  // Extract linear and angular velocities from the message
  double linear_x = (double)msg.linear.x;   // Linear velocity in m/s
  double angular_z = (double)msg.angular.z; // Angular velocity in rad/s

  // Multiply the angular speed due to mechanical properties of the robot
  angular_z *= 4; // Adjust this factor based on your robot's design

  // Convert velocities to motor speeds using differential drive kinematics
  double wheel_base = 0.235;                 // Distance between wheels (meters)
  double wheel_radius = WHEEL_RADIUS_METERS; // Radius of the wheels (meters)

  // Compute individual wheel speeds in RPS
  double left_front_speed_rps =
      ((-0.122 * angular_z) + linear_x) / WHEEL_RADIUS_METERS;
  double left_back_speed_rps =
      ((-0.125 * angular_z) + linear_x) / WHEEL_RADIUS_METERS;
  double right_back_speed_rps =
      ((0.141 * angular_z) + linear_x) / WHEEL_RADIUS_METERS;
  double right_front_speed_rps =
      ((0.141 * angular_z) + linear_x) / WHEEL_RADIUS_METERS;

  double left_front_speed_rpm = left_front_speed_rps * (60.0 / (2 * M_PI));
  double left_back_speed_rpm = left_back_speed_rps * (60.0 / (2 * M_PI));
  double right_front_speed_rpm = right_front_speed_rps * (60.0 / (2 * M_PI));
  double right_back_speed_rpm = right_back_speed_rps * (60.0 / (2 * M_PI));

  rpm_msg.x = left_front_speed_rpm;
  rpm_msg.y = right_front_speed_rpm;

  rcl_ret_t ret = rcl_publish(&rpm_publisher, &rpm_msg, NULL);

  // Send the desired RPM values to the motors using PID control

  if (Ps4.getButtonClick(START)) {
    robo.is_on = !robo.is_on;
  }

  if (robo.is_on) {
    Ps4.setLed(255, 0, 0); // Set LED to red when the robot is on
    btd_vhci_mutex_lock();
    ESP_LOGI("teste", "to aqui na task do controle");
    robo.controll_robot();
    btd_vhci_mutex_unlock();
  } else {
    Ps4.setLed(0, 0, 255); // Set LED to blue when the robot is off
    left_front_motor.move_pid(left_front_speed_rpm);
    left_back_motor.move_pid(left_back_speed_rpm);
    right_front_motor.move_pid(right_front_speed_rpm);
    right_back_motor.move_pid(right_back_speed_rpm);
  }

  // Delay to allow the FreeRTOS task to yield
  // vTaskDelay(pdMS_TO_TICKS(10));
}

void test_motor_working(void *task_params) {
  bool incrementando = true;
  int vel = 0;
  int posPrev = 0;
  long prevT = esp_timer_get_time();
  float angle = 0;
  int flag = 0;
  while (1) {
    //    left_front_motor.fetch_rpm();
    // left_back_motor.fetch_rpm();
    // right_front_motor.fetch_rpm();
    // right_back_motor.fetch_rpm();
    // long currT = esp_timer_get_time();
    // float deltaT = ((float)(currT - prevT)) / 1.0e6;
    // float posi = left_front_motor.posi;
    // float velocity = (posi - posPrev) / deltaT;
    float velocity_lf = left_front_motor.current_speed_rpm;
    float velocity_lb = left_back_motor.current_speed_rpm;
    float velocity_rf = right_front_motor.current_speed_rpm;
    float velocity_rb = right_back_motor.current_speed_rpm;
    float aaa = 0;

    if (flag < 200) {
      aaa = 30;
      flag++;
      incrementando = false;
    } else if (flag >= 200 && flag < 400) {
      aaa = -30;
      flag++;
      incrementando = false;
    }
    if (flag >= 400) {
      flag = 0;
    }

    int b = left_front_motor.posi;
    ESP_LOGI("velocidade_lf", "RPM: %f, erro: %f, Target %f, PWM %d",
             velocity_lf, left_front_motor.error, aaa, left_front_motor.pwm);
    ESP_LOGI("velocidade_lb", "RPM: %f, erro: %f, Target %f, PWM %d",
             velocity_lb, left_back_motor.error, aaa, left_back_motor.pwm);
    ESP_LOGI("velocidade_rf", "RPM: %f, erro: %f Target %f, PWM %d",
             velocity_rf, right_front_motor.error, aaa, right_front_motor.pwm);
    ESP_LOGI("velocidade_rb", "RPM: %f, erro: %f Target %f, PWM %d",
             velocity_rb, right_back_motor.error, aaa, right_back_motor.pwm);
    left_front_motor.move_pid(100);
    left_back_motor.move_pid(100);
    right_front_motor.move_pid(100);
    right_back_motor.move_pid(100);

    // left_back_motor.move_pid(vel);

    // right_front_motor.move_pid(vel);
    // right_back_motor.move_pid(vel);
    // ESP_LOGI("videos", "posi % rpm %" PRId32, PRId32, left_front_motor.posi,
    //          left_front_motor.return_speed());

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// PS4BT Ps4;
// RobotPs4Controller robo(&right_front_motor, &right_back_motor,
//                         &left_front_motor, &left_back_motor);

void rosStuff(void *task_params) {
  rclc_executor_t executor;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  rcl_node_t node;
  RCCHECK(rclc_node_init_default(&node, "int32_subscriber_rclc", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
      &rpm_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "motor_rpm"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
      &subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel_nav"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg,
                                         &subscription_callback, ALWAYS));
  while (1) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
    task_velocity();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void ps4_controller_task(void *task_params) {
  robo.task_robot_controll(task_params);
}

#define USARCONTROLE false
extern "C" void app_main(void) {
  esp_log_level_set("*", ESP_LOG_NONE);
  robot_setup();

  if (USARCONTROLE) {
    esp_err_t ret;
    ret = nvs_flash_init();
    ret = btd_vhci_init();
    btd_vhci_autoconnect(&Ps4);
    robo.set_controller(&Ps4);
  }

#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
  rmw_uros_set_custom_transport(true, (void *)&uart_port, esp32_serial_open,
                                esp32_serial_close, esp32_serial_write,
                                esp32_serial_read);
#else
// #error micro-ROS transports misconfigured
#endif // RMW_UXRCE_TRANSPORT_CUSTOM

  // xTaskCreate(RPM_fetching, "RPM_fetching", 4 * 1024, NULL, 1, NULL);
  xTaskCreate(rosStuff, "task-ros", 4 * 1024, NULL, 1, NULL);

  // xTaskCreate(task_velocity, "task_velocity", 4 * 1024, NULL, 1, NULL);
  // xTaskCreate(test_motor_working, "test_motor_working", 2 * 1024, NULL, 1,
  //             NULL);

  // vTaskDelete(NULL);
}
