idf_component_register(
    SRCS "EspRaspRobot.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp32_serial_transport MotorDC driver micro_ros_espidf_component
    PRIV_REQUIRES nvs_flash
)
