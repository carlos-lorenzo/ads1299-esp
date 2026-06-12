#include "ads1299.h"
#include "ads1299_defs.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "SPI_LOOPBACK";

extern "C" void app_main(void)
{

    gpio_num_t mosi_pin = GPIO_NUM_14;
    gpio_num_t miso_pin = GPIO_NUM_9;
    gpio_num_t sclk_pin = GPIO_NUM_10;

    spi_bus_config_t bus_cfg = {};

    bus_cfg.mosi_io_num = mosi_pin;
    bus_cfg.miso_io_num = miso_pin;
    bus_cfg.sclk_io_num = sclk_pin;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = ADS1299_FRAME_BYTES * 25;


    ESP_ERROR_CHECK(
        spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO)
    );


    ads1299_config_t config = {
        .spi_host = SPI2_HOST,
        .cs_pin = GPIO_NUM_11,
        .drdy_pin = GPIO_NUM_8,
        .reset_pin = GPIO_NUM_13,
        .start_pin = GPIO_NUM_12,
        .sample_rate = ADS1299_DR_500SPS,
    };

    ads1299_t dev = ads1299_create(&config);
    ads1299_init(&dev);


    // Temporal code for testing and making sure it isn't broken
    ESP_LOGI(TAG, "SPI Loopback Test Initialized. Short MISO (GPIO %d) and MOSI (GPIO %d)!", miso_pin, mosi_pin);

    // FIX: Dynamically allocate buffers from DMA-capable memory
    uint8_t* tx_buf = (uint8_t*)heap_caps_malloc(5, MALLOC_CAP_DMA);
    uint8_t* rx_buf = (uint8_t*)heap_caps_malloc(5, MALLOC_CAP_DMA);

    // Initialize test pattern
    tx_buf[0] = 0xAA; tx_buf[1] = 0x55; tx_buf[2] = 0x11; tx_buf[3] = 0x22; tx_buf[4] = 0x33;

    spi_transaction_t t = {};
    t.length = 5 * 8;
    t.rxlength = 5 * 8;
    t.tx_buffer = tx_buf;
    t.rx_buffer = rx_buf;

    esp_err_t ret;

    while (true) {
        // Clear RX buffer before every transfer
        memset(rx_buf, 0, 5);

        // Manually drive Chip Select LOW
        gpio_set_level(config.cs_pin, 0);
        esp_rom_delay_us(2);

        // Execute transmission
        ret = spi_device_polling_transmit(dev.spi_handle, &t);

        // Manually drive Chip Select HIGH
        esp_rom_delay_us(2);
        gpio_set_level(config.cs_pin, 1);

        if (ret == ESP_OK) {
            // Check if Received Data perfectly mirrors Transmitted Data
            if (memcmp(tx_buf, rx_buf, 5) == 0) {
                ESP_LOGI(TAG, "Loopback SUCCESS! Received: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
                         rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4]);
            } else {
                ESP_LOGE(TAG, "Loopback MISMATCH! Sent data didn't return. Received: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
                         rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4]);
            }
        } else {
            ESP_LOGE(TAG, "SPI Transaction Failed!");
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    // Clean up heap allocation if you ever exit this loop
    free(tx_buf);
    free(rx_buf);

}



