idf_component_register(
    SRCS "ads1299.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_driver_spi esp_driver_gpio
)
