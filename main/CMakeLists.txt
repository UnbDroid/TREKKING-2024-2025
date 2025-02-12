idf_component_register(SRCS "main.cpp" 
                    "../components/MotorDC/MotorDC.cpp"
                    "../components/PinConfig/PinConfig.cpp"
                    INCLUDE_DIRS ".")

set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/components Ps4Controller)
