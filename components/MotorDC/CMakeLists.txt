idf_component_register(SRCS "MotorDC.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES driver
                    PRIV_REQUIRES PinConfig
                    )
