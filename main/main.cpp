#include <stdio.h>
#include "ads1299.h"
#include "ads1299_defs.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "SPI_LOOPBACK";

// Define your physical pins based on your ESP32 model wiring
#define TEST_SPI_HOST    SPI2_HOST
#define PIN_NUM_MISO     GPIO_NUM_19
#define PIN_NUM_MOSI     GPIO_NUM_23
#define PIN_NUM_CLK      GPIO_NUM_18
#define PIN_NUM_CS       GPIO_NUM_5

extern "C" void app_main(void)
{
    esp_err_t ret;
    ads1299_config_t config = {
        .spi_handle = nullptr, // Will be set by ads1299_init
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


    ESP_LOGI(TAG, "SPI Loopback Test Initialized. Short MISO (GPIO %d) and MOSI (GPIO %d)!", config.miso_pin, config.mosi_pin);

    // 4. Test Buffers
    uint8_t tx_buf[5] = {0xAA, 0x55, 0x11, 0x22, 0x33}; // Test pattern
    uint8_t rx_buf[5] = {0};

    spi_transaction_t t = {
        .flags = 0,
        .cmd = 0,
        .addr = 0,
        .length = 5 * 8, // 5 bytes = 40 bits
        .rxlength = 5 * 8, // 5 bytes = 40 bits
        .override_freq_hz = 0, // Use default frequency
        .user = nullptr,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };

    while (1) {
        // Clear RX buffer before every transfer to ensure no false positives
        memset(rx_buf, 0, sizeof(rx_buf));

        // Manually drive Chip Select LOW
        gpio_set_level(config.cs_pin, 0);
        esp_rom_delay_us(2); // Meet setup time

        // Execute transmission
        ret = spi_device_polling_transmit(config.spi_handle, &t);

        // Manually drive Chip Select HIGH
        esp_rom_delay_us(2); // Meet hold time
        gpio_set_level(config.cs_pin, 1);

        if (ret == ESP_OK) {
            // Check if Received Data perfectly mirrors Transmitted Data
            if (memcmp(tx_buf, rx_buf, sizeof(tx_buf)) == 0) {
                ESP_LOGI(TAG, "Loopback SUCCESS! Received: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
                         rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4]);
            } else {
                ESP_LOGE(TAG, "Loopback MISMATCH! Sent data didn't return. Received: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
                         rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4]);
            }
        } else {
            ESP_LOGE(TAG, "SPI Transaction Failed!");
        }

        vTaskDelay(pdMS_TO_TICKS(2000)); // Repeat every 2 seconds
    }
}

