#include "MotorDC.h"
#include "RobotPs4Controller.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "PS4BT.h"
#include "btd_vhci.h"
PS4BT PS4;
RobotPs4Controller robo;
void task_controll(void *task_params) { robo.task_robot_controll(task_params); }
extern "C" void app_main(void) {
  esp_err_t ret;

  // initialize flash
  ret = nvs_flash_init();
  ret = btd_vhci_init();
  btd_vhci_autoconnect(&PS4);
  robo.set_controller(&PS4);
  xTaskCreatePinnedToCore(task_controll, "ps4_loop_task", 10 * 1024, NULL, 2,
                          NULL, 1);
}
