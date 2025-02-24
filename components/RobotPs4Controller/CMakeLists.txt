idf_component_register(SRCS "RobotPs4Controller.cpp"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES MotorDC nvs_flash
                    REQUIRES pink0d__btd_vhci
                  )
