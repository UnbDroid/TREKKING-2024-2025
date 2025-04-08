#include "EspRaspRobot.h"

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
        double left_speed = state.rpm_left_velocity_mean;
        double right_speed = state.rpm_right_velocity_mean;
        double linear_velocity = (left_speed + right_speed) / 2.0;
        
        // Compute angular velocity from differential drive kinematics (ensure wheel_base is set)
        double angular_velocity = (right_speed - left_speed) / 0.275;

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

        RCSOFTCHECK(rcl_publish(&this->odom_publisher, &this->odom, NULL));
    }
}

void EspRaspRobot::timer_callback_wrapper(rcl_timer_t* timer, int64_t last_call_time)
{
    if (EspRaspRobot::instance != NULL) {
        EspRaspRobot::instance->timer_callback(timer, last_call_time);
    }
}

void EspRaspRobot::update_posi_and_speed() {
    RoboVirtual state = this->robotProperties->compute_vector_position();
    
    this->pos_x = state.vectorPosition.x;
    this->pos_y = state.vectorPosition.y;
    this->yaw   = state.vectorPosition.anguloTheta;
    
    this->speed_left_front  = state.rpm_left_velocity_mean;
    this->speed_left_back   = state.rpm_left_velocity_mean;
    this->speed_right_front = state.rpm_right_velocity_mean;
    this->speed_right_back  = state.rpm_right_velocity_mean;
}

void EspRaspRobot::micro_ros_run() {
    // Initialize the node

    RCCHECK(rclc_support_init(&this->support, 0, NULL, &this->allocator));

    rclc_node_init_default(&this->esp_node, "esp32_node", "", &this->support);

    // Create the publisher for odometry
    rclc_publisher_init_default(
        &this->odom_publisher,
        &this->esp_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom");

    // Create the timer for publishing odometry
    const unsigned int timer_timeout = 1000;
    RCCHECK(rclc_timer_init_default(
		&timer,
		&this->support,
		RCL_MS_TO_NS(timer_timeout),
		EspRaspRobot::timer_callback_wrapper));

    // Create the executor
    RCCHECK(rclc_executor_init(&this->executor, &this->support.context, 1, &this->allocator));
    RCCHECK(rclc_executor_add_timer(&this->executor, &this->timer));

    // Spin the node
    while (1) {
        rclc_executor_spin_some(&this->executor, RCL_MS_TO_NS(100));
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // free resources
    RCCHECK(rcl_publisher_fini(&this->odom_publisher, &this->esp_node));
    RCCHECK(rcl_node_fini(&this->esp_node));

    vTaskDelete(NULL);
}

void EspRaspRobot::follow_path() {
    int desired_speed_left = this->desired_speed_left_vol;
    int desired_speed_right = this->desired_speed_right_vol;
    this->left_front_motor->go_forward(desired_speed_left);
    this->right_front_motor->go_forward(desired_speed_right);
    this->left_back_motor->go_forward(desired_speed_left);
    this->right_back_motor->go_forward(desired_speed_right);
}