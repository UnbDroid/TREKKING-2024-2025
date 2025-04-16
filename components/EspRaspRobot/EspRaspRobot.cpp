#include "EspRaspRobot.h"
#include "rosidl_runtime_c/string_functions.h"

EspRaspRobot* EspRaspRobot::instance = nullptr;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

static size_t uart_port = UART_NUM_0;

EspRaspRobot::EspRaspRobot(MotorDC *right_front_motor,
                           MotorDC *right_back_motor,
                           MotorDC *left_front_motor,
                           MotorDC *left_back_motor,
                           RobotProperties *robotProperties) 

{
    this->right_front_motor = right_front_motor;
    this->right_back_motor = right_back_motor;
    this->left_front_motor = left_front_motor;
    this->left_back_motor = left_back_motor;
    this->robotProperties = robotProperties;
    instance = this;
};

void EspRaspRobot::micro_ros_setup() {
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
}

void EspRaspRobot::timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        // Step 1: Retrieve the current state from robotProperties.
        // RoboVirtual should include:
        // - rpm_left_velocity_mean, rpm_right_velocity_mean
        // - vectorPosition with x, y and anguloTheta (yaw)
        RoboVirtual state = this->robotProperties->compute_vector_position();
        double left_speed = state.rpm_left_velocity_mean*WHEEL_RADIUS_METERS;
        double right_speed = state.rpm_right_velocity_mean*WHEEL_RADIUS_METERS;
        double linear_velocity = (left_speed + right_speed) / 2.0;
        
        // Compute angular velocity from differential drive kinematics (ensure wheel_base is set)
        double angular_velocity = (left_speed - right_speed) / (0.275/2);

        this->odom.pose.pose.position.x = state.vectorPosition.x;
        this->odom.pose.pose.position.y = state.vectorPosition.y;
        this->odom.pose.pose.position.z = 0.0;
        
        geometry_msgs__msg__Quaternion quat;
        double yaw = state.vectorPosition.anguloTheta;
        quat.w = cos(yaw * 0.5);
        quat.x = 0.0;
        quat.y = 0.0;
        quat.z = sin(yaw * 0.5);
        this->odom.pose.pose.orientation = quat;
        
        this->odom.twist.twist.linear.x  = linear_velocity;  // in whatever unit you calculate
        this->odom.twist.twist.linear.y  = 0.0;
        this->odom.twist.twist.linear.z  = 0.0;
        this->odom.twist.twist.angular.z = angular_velocity;
        this->odom.twist.twist.angular.x = 0.0;
        this->odom.twist.twist.angular.y = 0.0;

        // Set the frame_id and child_frame_id
        rosidl_runtime_c__String__assign(&this->odom.header.frame_id, "odom");
        rosidl_runtime_c__String__assign(&this->odom.child_frame_id, "base_link");

        RCSOFTCHECK(rcl_publish(&this->odom_publisher, &this->odom, NULL));

        // Now the subscription

    }
}

void EspRaspRobot::timer_callback_wrapper(rcl_timer_t* timer, int64_t last_call_time)
{
    if (EspRaspRobot::instance != NULL) {
        EspRaspRobot::instance->timer_callback(timer, last_call_time);
    }
}

void EspRaspRobot::subscription_callback(const void * msgin)
{
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

    // Extract linear and angular velocities from the message
    double linear_x = msg->linear.x;  // Forward/backward velocity
    double angular_z = msg->angular.z;  // Rotational velocity

    // Convert velocities to motor speeds using differential drive kinematics
    double wheel_base = 0.275;  // Distance between wheels (meters)
    double wheel_radius = WHEEL_RADIUS_METERS;

    // Compute individual wheel speeds
    double left_speed = (linear_x - (angular_z * wheel_base / 2)) / wheel_radius;
    double right_speed = (linear_x + (angular_z * wheel_base / 2)) / wheel_radius;

    // Convert speeds to PWM values (assuming a linear relationship)
    this->desired_speed_left_vol = static_cast<int>(left_speed * 100);  // Scale factor for PWM
    this->desired_speed_right_vol = static_cast<int>(right_speed * 100);

    // Apply the computed speeds to the motors
    this->follow_path();
}

void EspRaspRobot::subscription_callback_wrapper(const void * msgin)
{
    if (EspRaspRobot::instance != NULL) {
        EspRaspRobot::instance->subscription_callback(msgin);
    }
}

void EspRaspRobot::micro_ros_run() {
    // Initialize the node

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

    RCCHECK(rcl_init_options_init(&init_options, this->allocator));

    RCCHECK(rclc_support_init_with_options(&this->support, 0, NULL, &init_options, &this->allocator));

    rclc_node_init_default(&this->esp_node, "esp32_node", "", &this->support);

    // Create the publisher for odometry
    RCCHECK(rclc_publisher_init_default(
        &this->odom_publisher,
        &this->esp_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom"));

    // Create the subscription for cmd_vel
    RCCHECK(rclc_subscription_init_default(
        &this->subscription,
        &this->esp_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    // Create the timer for publishing odometry
    const unsigned int timer_timeout = 10;
    RCCHECK(rclc_timer_init_default(
		&timer,
		&this->support,
		RCL_MS_TO_NS(timer_timeout),
		EspRaspRobot::timer_callback_wrapper));

    // Create the executor
    RCCHECK(rclc_executor_init(&this->executor, &this->support.context, 1, &this->allocator));
    RCCHECK(rclc_executor_set_timeout(&this->executor, RCL_MS_TO_NS(this->timer_timeout)));
    RCCHECK(rclc_executor_add_timer(&this->executor, &this->timer));
    RCCHECK(rclc_executor_add_subscription(&this->executor, &this->subscription,
                                          &this->cmd_vel,
                                          EspRaspRobot::subscription_callback_wrapper,
                                          ON_NEW_DATA));

    // Spin the node
    while (1) {
        rclc_executor_spin_some(&this->executor, RCL_MS_TO_NS(10));
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // free resources
    RCCHECK(rcl_publisher_fini(&this->odom_publisher, &this->esp_node));
    RCCHECK(rcl_subscription_fini(&this->subscription, &this->esp_node));
    RCCHECK(rcl_node_fini(&this->esp_node));

    vTaskDelete(NULL);
}

void EspRaspRobot::follow_path() {
    int desired_speed_left = this->desired_speed_left_vol;
    int desired_speed_right = this->desired_speed_right_vol;

    // Apply the speeds to the motors
    this->left_front_motor->move_pid(desired_speed_left);
    this->right_front_motor->move_pid(desired_speed_right);
    this->left_back_motor->move_pid(desired_speed_left);
    this->right_back_motor->move_pid(desired_speed_right);
}