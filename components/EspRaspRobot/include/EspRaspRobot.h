#ifndef ESP_RASP_ROBOT
#define ESP_RASP_ROBOT

#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "MotorDC.h"
#include "RobotProperties.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <math.h>
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"

class EspRaspRobot {
    public:
        // Constructor that accepts 4 MotorDC pointers
        EspRaspRobot(MotorDC *left_front_motor, MotorDC *right_front_motor, MotorDC *left_back_motor, MotorDC *right_back_motor, RobotProperties *robotProperties);

        void micro_ros_setup();
        static void timer_callback_wrapper(rcl_timer_t* timer, int64_t last_call_time);
        static void subscription_callback_wrapper(const void * msgin);
        void micro_ros_run();
        void follow_path();
        void test_micro_ros();

    private:
        MotorDC* left_front_motor;
        MotorDC* right_front_motor;
        MotorDC* left_back_motor;
        MotorDC* right_back_motor;
        RobotProperties* robotProperties;
        static EspRaspRobot* instance;
        
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

        int desired_speed_left_vol = 0;
        int desired_speed_right_vol = 0;

        // Main Node
        rcl_node_t esp_node = rcl_get_zero_initialized_node();

        // Pub and Sub
        rcl_publisher_t odom_publisher;
        rcl_subscription_t subscription;
        
        // Allocator, support and executor
        rcl_allocator_t allocator = rcl_get_default_allocator();
        rclc_support_t support;
        rclc_executor_t executor;

        // Msg 
        nav_msgs__msg__Odometry odom;

        // Timer Stuff
        rcl_timer_t timer;
    	const unsigned int timer_timeout = 1000;

        // Received message
        geometry_msgs__msg__Twist cmd_vel_nav;

        // Functions
        void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
        void subscription_callback(const void * msgin);

};

#endif