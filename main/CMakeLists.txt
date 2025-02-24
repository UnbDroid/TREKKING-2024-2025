idf_component_register(SRCS "main.cpp" 
                    "../components/MotorDC/MotorDC.cpp"
                    "../components/PinConfig/PinConfig.cpp"
                    #"../components/Ps4Controller/src/PS4Controller.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES  MotorDC PinConfig
                    PRIV_REQUIRES nvs_flash 
                  )

#set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/components Ps4Controller)
