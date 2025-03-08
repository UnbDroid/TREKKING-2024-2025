idf_component_register(SRCS "main.cpp" 
                    "../components/MotorDC/MotorDC.cpp"
                    "../components/PinConfig/PinConfig.cpp"
                    "../components/RobotPs4Controller/RobotPs4Controller.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES  MotorDC PinConfig RobotPs4Controller
                    PRIV_REQUIRES nvs_flash 
                  )

