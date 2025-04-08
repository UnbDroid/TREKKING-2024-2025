idf_component_register(SRCS "main.cpp" 
                    "../components/MotorDC/MotorDC.cpp"
                    "../components/PinConfig/PinConfig.cpp"
                    "../components/RobotPs4Controller/RobotPs4Controller.cpp"
                    "../components/RobotProperties/RobotProperties.cpp"
                    "../components/EspRaspRobot/EspRaspRobot.cpp"
                    "../components/esp32_serial_transport/esp32_serial_transport.c"
                    INCLUDE_DIRS "."
                    REQUIRES  MotorDC PinConfig RobotPs4Controller RobotProperties EspRaspRobot
                    PRIV_REQUIRES nvs_flash driver micro_ros_espidf_component
                  )

