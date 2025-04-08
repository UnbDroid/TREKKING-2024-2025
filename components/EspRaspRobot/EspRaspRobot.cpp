#include "EspRaspRobot.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

static size_t uart_port = UART_NUM_0;

EspRaspRobot::EspRaspRobot(MotorDC *right_front_motor,
                           MotorDC *right_back_motor,
                           MotorDC *left_front_motor,
                           MotorDC *left_back_motor)
{
    this->right_front_motor = right_front_motor;
    this->right_back_motor = right_back_motor;
    this->left_front_motor = left_front_motor;
    this->left_back_motor = left_back_motor;
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

void EspRaspRobot::update_posi_and_speed() { // Should be made a task
    this->posi_left_front = ((left_front_motor->return_posi() / 300) * left_front_motor->wheel_lenght);
    this->posi_right_front = ((right_front_motor->return_posi() / 300) * right_front_motor->wheel_lenght);
    this->posi_left_back = ((left_back_motor->return_posi() / 300) * left_back_motor->wheel_lenght);
    this->posi_right_back = ((right_back_motor->return_posi() / 300) * right_back_motor->wheel_lenght);
    this->speed_left_front = this->left_front_motor->return_speed();
    this->speed_right_front = this->right_front_motor->return_speed();
    this->speed_left_back = this->left_back_motor->return_speed();
    this->speed_right_back = this->right_back_motor->return_speed();
}

void EspRaspRobot::go_forward(double distance) {
    int32_t initial_posi_left_front = this->posi_left_front;
    int32_t initial_posi_right_front = this->posi_right_front;
    int32_t initial_posi_left_back = this->posi_left_back;
    int32_t initial_posi_right_back = this->posi_right_back;
    while ((this->posi_left_front - initial_posi_left_front) > (int32_t)distance or
           (this->posi_right_front - initial_posi_right_front) > (int32_t)distance or
           (this->posi_left_back - initial_posi_left_back) > (int32_t)distance or
           (this->posi_right_back - initial_posi_right_back) > (int32_t)distance)
    {

        this->left_front_motor->go_forward(150);
        this->right_front_motor->go_forward(150);
        this->left_back_motor->go_forward(150);
        this->right_back_motor->go_forward(150);
    }
}

void EspRaspRobot::follow_path() {
    int desired_speed_left = this->desired_speed_left_vol;
    int desired_speed_right = this->desired_speed_right_vol;
    this->left_front_motor->go_forward(desired_speed_left);
    this->right_front_motor->go_forward(desired_speed_right);
    this->left_back_motor->go_forward(desired_speed_left);
    this->right_back_motor->go_forward(desired_speed_right);
}