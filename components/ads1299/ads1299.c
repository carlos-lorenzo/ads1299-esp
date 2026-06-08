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
    // WREG command: two command bytes followed by the register value.
    // Byte 0: 0x40 | reg address
    // Byte 1: number of registers to write minus 1 (0x00 = write 1 register)
    // Byte 2: value
    uint8_t tx[3] = {
        (uint8_t)(ADS1299_CMD_WREG | (reg & 0x1F)),
        0x00,
        value,
    };

    gpio_set_level(dev->config.cs_pin, 0);
    esp_rom_delay_us(ADS1299_T_CSSC);

    spi_transaction_t t = {
        .length = 8 * 3,
        .tx_buffer = tx,
        .rx_buffer = nullptr,
    };

    esp_err_t ret = spi_device_polling_transmit(dev->spi_handle, &t);

    esp_rom_delay_us(ADS1299_T_CSH);
    gpio_set_level(dev->config.cs_pin, 1);

    return ret;
}

esp_err_t ads1299_read_register(ads1299_t* dev, uint8_t reg, uint8_t* value)
{
    // RREG command: two command bytes, then clock out one byte to receive.
    // Byte 0: 0x20 | reg address
    // Byte 1: number of registers to read minus 1 (0x00 = read 1 register)
    // Byte 2: dummy TX byte — chip drives MISO with the register value
    uint8_t tx[3] = {
        (uint8_t)(ADS1299_CMD_RREG | (reg & 0x1F)),
        0x00,
        0x00,
    };
    uint8_t rx[3] = {0};

    gpio_set_level(dev->config.cs_pin, 0);
    esp_rom_delay_us(ADS1299_T_CSSC);

    spi_transaction_t t = {
        .length = 8 * 3,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    esp_err_t ret = spi_device_polling_transmit(dev->spi_handle, &t);

    esp_rom_delay_us(ADS1299_T_CSH);
    gpio_set_level(dev->config.cs_pin, 1);

    if (ret == ESP_OK) {
        *value = rx[2];
    }

    return ret;
}

esp_err_t ads1299_write_registers(ads1299_t* dev, uint8_t start_reg, const uint8_t* data, size_t count)
{
    // Same as write_register but byte 1 is (count - 1) and followed by count value bytes
    uint8_t tx[2 + count];
    tx[0] = (uint8_t)(ADS1299_CMD_WREG | (start_reg & 0x1F));
    tx[1] = (uint8_t)(count - 1);
    for (size_t i = 0; i < count; i++) {
        tx[2 + i] = data[i];
    }

    gpio_set_level(dev->config.cs_pin, 0);
    esp_rom_delay_us(ADS1299_T_CSSC);

    spi_transaction_t t = {
        .length = 8 * (2 + count),
        .tx_buffer = tx,
        .rx_buffer = nullptr,
    };

    esp_err_t ret = spi_device_polling_transmit(dev->spi_handle, &t);

    esp_rom_delay_us(ADS1299_T_CSH);
    gpio_set_level(dev->config.cs_pin, 1);

    return ret;
}

esp_err_t ads1299_read_registers(ads1299_t* dev, uint8_t start_reg, uint8_t* data, size_t count)
{
    // Same as read_register but byte 1 is (count - 1) and count bytes are received after
    uint8_t tx[2 + count];
    tx[0] = (uint8_t)(ADS1299_CMD_RREG | (start_reg & 0x1F));
    tx[1] = (uint8_t)(count - 1);
    for (size_t i = 0; i < count; i++) {
        tx[2 + i] = 0x00;
    }

    uint8_t rx[2 + count];

    gpio_set_level(dev->config.cs_pin, 0);
    esp_rom_delay_us(ADS1299_T_CSSC);

    spi_transaction_t t = {
        .length = 8 * (2 + count),
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    esp_err_t ret = spi_device_polling_transmit(dev->spi_handle, &t);

    esp_rom_delay_us(ADS1299_T_CSH);
    gpio_set_level(dev->config.cs_pin, 1);

    if (ret == ESP_OK) {
        for (size_t i = 0; i < count; i++) {
            data[i] = rx[2 + i];
        }
    }

    return ret;
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
    if (!dev || !sample) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buffer[ADS1299_FRAME_SIZE];

    esp_err_t ret = ads1299_read_data(dev, buffer);
    if (ret != ESP_OK) {
        return ret;
    }

    // First 3 bytes are status
    sample->status[0] = buffer[0];
    sample->status[1] = buffer[1];
    sample->status[2] = buffer[2];

    // Remaining 24 bytes are 8 channels of 3 bytes each, 24-bit signed MSB first
    for (int ch = 0; ch < 8; ch++) {
        int offset = 3 + ch * 3;
        sample->channels[ch] = ((int32_t)(int8_t)buffer[offset] << 16)
                              | ((uint32_t)buffer[offset + 1] << 8)
                              |  (uint32_t)buffer[offset + 2];
    }

    return ESP_OK;
}

esp_err_t ads1299_start(ads1299_t* dev)
{
    gpio_set_level(dev->config.start_pin, 1);
    return ESP_OK;
}

esp_err_t ads1299_stop(ads1299_t* dev)
{
    gpio_set_level(dev->config.start_pin, 0);
    return ads1299_send_command(dev, ADS1299_CMD_STOP);
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
    return ads1299_send_command(dev, ADS1299_CMD_STANDBY);
}

esp_err_t ads1299_wakeup(ads1299_t* dev)
{
    esp_err_t ret = ads1299_send_command(dev, ADS1299_CMD_WAKEUP);
    esp_rom_delay_us(ADS1299_TICKS_TO_US(4));
    return ret;
}

esp_err_t ads1299_enable_continuous_read(ads1299_t* dev)
{
    esp_err_t ret = ads1299_send_command(dev, ADS1299_CMD_RDATAC);
    esp_rom_delay_us(ADS1299_T_SDATAC);
    return ret;
}

esp_err_t ads1299_disable_continuous_read(ads1299_t* dev)
{
    esp_err_t ret = ads1299_send_command(dev, ADS1299_CMD_SDATAC);
    esp_rom_delay_us(ADS1299_T_SDATAC);
    return ret;
}
