#ifndef ESP_RASP_ROBOT
#define ESP_RASP_ROBOT

#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "MotorDC.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/byte_multi_array.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"

class EspRaspRobot {
    public:
        // Constructor that accepts 4 MotorDC pointers
        EspRaspRobot(MotorDC *left_front_motor, MotorDC *right_front_motor, MotorDC *left_back_motor, MotorDC *right_back_motor);

        void micro_ros_setup();
        void micro_ros_run();
        void update_posi_and_speed();
        void go_forward(double distance);
        void follow_path();
        void test_micro_ros();

    private:
        MotorDC* left_front_motor;
        MotorDC* right_front_motor;
        MotorDC* left_back_motor;
        MotorDC* right_back_motor;
        
        double pos_x;
        double pos_y;
        double yaw;

        volatile int32_t posi_left_front = 0;
        volatile int32_t posi_right_front = 0;
        volatile int32_t posi_left_back = 0;
        volatile int32_t posi_right_back = 0;

        volatile double speed_left_front = 0;
        volatile double speed_right_front = 0;
        volatile double speed_left_back = 0;
        volatile double speed_right_back = 0;

        volatile int desired_speed_left_vol = 0;
        volatile int desired_speed_right_vol = 0;

        // Main Node
        rcl_node_t esp_node;

        // Motor Speed Publishers
        rcl_publisher_t left_motor_speed_publisher;
        rcl_publisher_t right_motor_speed_publisher;

        // Motor Odometry Publisher
        rcl_publisher_t left_motor_odometry_publisher;
        rcl_publisher_t right_motor_odometry_publisher;

        // Map Position Subscriber
        rcl_subscription_t map_position_subscriber;

        // Desired Speed Subscriber
        rcl_subscription_t left_desired_speed_subscriber;
        rcl_subscription_t right_desired_speed_subscriber;

        // Timer Stuff
        rcl_timer_t timer;
    	const unsigned int timer_timeout = 1000;

};

#endif