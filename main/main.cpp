#include <stdio.h>
#include "ads1299.h"
#include "ads1299_defs.h"

extern "C" void app_main(void)
{
    ads1299_config_t config = {
        .spi_host = SPI2_HOST,
        .cs_pin = GPIO_NUM_11,
        .mosi_pin = GPIO_NUM_14,
        .miso_pin = GPIO_NUM_9,
        .sclk_pin = GPIO_NUM_10,
        .drdy_pin = GPIO_NUM_8,
        .reset_pin = GPIO_NUM_13,
        .start_pin = GPIO_NUM_12,
        .sample_rate = ADS1299_DR_500SPS,
    };

    ads1299_init(&config);
}

