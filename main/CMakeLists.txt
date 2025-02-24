idf_component_register(SRCS "main.cpp" 
                    "../components/MotorDC/MotorDC.cpp"
                    "../components/PinConfig/PinConfig.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES  MotorDC PinConfig
                    PRIV_REQUIRES nvs_flash 
                  )

