#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ads1299.h"

static const char *TAG = "ADS1299";

ads1299_t ads1299_create(const ads1299_config_t *cfg)
{
    ads1299_t dev = {0};
    if (cfg) {
        dev.config = *cfg;
    }
    return dev;
}

/* init */
esp_err_t ads1299_init(ads1299_t *dev)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }

    if (dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = ESP_OK;

    ESP_LOGI(TAG, "Initializing ADS1299");

    /* ---------------- GPIO setup ---------------- */
    gpio_config_t io_cfg = {
        .pin_bit_mask =
            (1ULL << dev->config.cs_pin) |
            (1ULL << dev->config.reset_pin) |
            (1ULL << dev->config.start_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_RETURN_ON_ERROR(gpio_config(&io_cfg), TAG, "GPIO config failed");

    gpio_set_level(dev->config.cs_pin, 1);
    gpio_set_level(dev->config.reset_pin, 1);
    gpio_set_level(dev->config.start_pin, 0);

    /* ---------------- SPI bus ---------------- */

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = dev->config.mosi_pin,
        .miso_io_num = dev->config.miso_pin,
        .sclk_io_num = dev->config.sclk_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64
    };

    ESP_RETURN_ON_ERROR(
        spi_bus_initialize(dev->config.spi_host, &bus_cfg, SPI_DMA_CH_AUTO),
        TAG,
        "SPI bus init failed"
    );

    spi_device_interface_config_t dev_cfg = {
        .mode = 1,
        .clock_speed_hz = 4 * 1000 * 1000,
        .spics_io_num = -1,   // manual CS control
        .queue_size = 3,
    };

    ESP_RETURN_ON_ERROR(
        spi_bus_add_device(dev->config.spi_host, &dev_cfg, &dev->spi_handle),
        TAG,
        "SPI device add failed"
    );

    /* ---------------- driver state ---------------- */

    dev->mutex = xSemaphoreCreateMutex();
    if (!dev->mutex) {
        return ESP_ERR_NO_MEM;
    }

    /* ---------------- ADS1299 hardware reset sequence ---------------- */

    vTaskDelay(pdMS_TO_TICKS(10));

    gpio_set_level(dev->config.reset_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(dev->config.reset_pin, 1);

    vTaskDelay(pdMS_TO_TICKS(50));

    /* SDATAC (stop continuous mode) */
    // ads1299_send_command(dev, ADS1299_CMD_SDATAC);

    ESP_LOGI(TAG, "ADS1299 initialized");

    dev->initialized = true;

    return ret;
}


/* deinit */
esp_err_t ads1299_deinit(ads1299_t *dev)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Deinitializing ADS1299");

    /* stop device */
    gpio_set_level(dev->config.start_pin, 0);

    /* remove SPI device */
    if (dev->spi_handle) {
        spi_bus_remove_device(dev->spi_handle);
        dev->spi_handle = nullptr;
    }

    /* free SPI bus */
    spi_bus_free(dev->config.spi_host);

    /* free mutex */
    if (dev->mutex) {
        vSemaphoreDelete(dev->mutex);
        dev->mutex = nullptr;
    }

    dev->initialized = false;

    ESP_LOGI(TAG, "ADS1299 deinitialized");

    return ESP_OK;
}

esp_err_t ads1299_write_register(ads1299_t* dev, uint8_t reg, uint8_t value)
{
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t ads1299_read_register(ads1299_t* dev, uint8_t reg, uint8_t* value)
{
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t ads1299_write_registers(ads1299_t* dev, uint8_t start_reg, const uint8_t* data, size_t count)
{
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t ads1299_read_registers(ads1299_t* dev, uint8_t start_reg, uint8_t* data, size_t count)
{
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t ads1299_send_command(ads1299_t* dev, uint8_t command)
{
    // Assert CS pin
    gpio_set_level(dev->config.cs_pin, 0);
    esp_rom_delay_us(ADS1299_T_CSSC);

    // Send command byte over SPI
    spi_transaction_t t;
    t.length = 8;
    t.rxlength = 0;
    t.tx_buffer = &command;
    t.rx_buffer = nullptr;

    esp_err_t ret = spi_device_polling_transmit(dev->spi_handle, &t);

    esp_rom_delay_us(ADS1299_T_CSH);

    ESP_ERROR_CHECK(ret);

    // Deassert CS pin
    gpio_set_level(dev->config.cs_pin, 1);

    return ret;
}

esp_err_t ads1299_read_data(ads1299_t* dev, uint8_t* buffer)
{
    if (!dev || !buffer) {
        return ESP_ERR_INVALID_ARG;
    }

    gpio_set_level(dev->config.cs_pin, 0);
    esp_rom_delay_us(ADS1299_T_CSSC);

    static const uint8_t cmd = ADS1299_CMD_RDATA;

    uint8_t tx[ADS1299_FRAME_SIZE] = {0};
    uint8_t rx[ADS1299_FRAME_SIZE] = {0};

    spi_transaction_t t = {
        .length = ADS1299_FRAME_SIZE * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    tx[0] = cmd;

    esp_err_t ret = spi_device_polling_transmit(dev->spi_handle, &t);

    esp_rom_delay_us(ADS1299_T_CSH);
    gpio_set_level(dev->config.cs_pin, 1);

    if (ret != ESP_OK) {
        return ret;
    }

    // copy only meaningful data
    memcpy(buffer, rx, ADS1299_FRAME_SIZE);

    return ESP_OK;
}

esp_err_t ads1299_read_sample(ads1299_t* dev, ads1299_sample_t* sample)
{
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t ads1299_start(ads1299_t* dev)
{
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t ads1299_stop(ads1299_t* dev)
{
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t ads1299_reset_hardware(ads1299_t* dev)
{
    esp_err_t ret = ESP_OK;
    ret = gpio_set_level(dev->config.reset_pin, 0);
    esp_rom_delay_us(ADS1299_T_RESET);
    gpio_set_level(dev->config.reset_pin, 1);

    return ret;
}

esp_err_t ads1299_reset_software(ads1299_t* dev)
{
    esp_err_t ret = ads1299_send_command(dev, ADS1299_CMD_RESET);
    esp_rom_delay_us(ADS1299_T_RESET);
    return ret;
}

esp_err_t ads1299_standby(ads1299_t* dev)
{
    return ESP_ERR_NOT_SUPPORTED;
}


esp_err_t ads1299_wakeup(ads1299_t* dev)
{
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t ads1299_enable_continuous_read(ads1299_t* dev)
{
    esp_err_t ret = ads1299_send_command(dev, ADS1299_CMD_RESET);
    esp_rom_delay_us(ADS1299_T_SDATAC);
    return ret;
}

esp_err_t ads1299_disable_continuous_read(ads1299_t* dev)
{
    return ESP_ERR_NOT_SUPPORTED;
}


//
// esp_err_t read_data(ads1299_config_t* config, uint8_t* buffer, size_t buffer_len)
// {
//     // This function will be called from the DRDY interrupt handler, so it should be designed to be as fast as possible. It will read the data registers from the ADS1299 and store the data in the provided buffer. The buffer should be large enough to hold the data for all 8 channels (24 bits per channel) plus any additional bytes for status or metadata if needed.
//
//     // Assert CS pin
//     gpio_set_level(config->cs_pin, 0);
//     esp_rom_delay_us(ADS1299_T_CSSC);
//
//     // Send RDATA command followed by dummy bytes to read the data
//     uint8_t command = ADS1299_CMD_RDATA;
//     spi_transaction_t t;
//     t.length = 8;
//     t.rxlength = buffer_len * 8;
//     t.tx_buffer = &command;
//     t.rx_buffer = buffer;
//
//     esp_err_t ret = spi_device_polling_transmit(config->spi_handle, &t);
//
//     esp_rom_delay_us(ADS1299_T_CSH);
//
//     ESP_ERROR_CHECK(ret);
//
//     // Deassert CS pin
//     gpio_set_level(config->cs_pin, 1);
//
//     return ret;
// }
//
//


