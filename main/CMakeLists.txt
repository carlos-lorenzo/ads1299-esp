idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES ads1299 esp_driver_spi esp_driver_gpio esp_ringbuf esp_timer
)
