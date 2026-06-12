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

    /* ---------------- ADS1299 hardware powerup sequence ---------------- */
    esp_err_t err;
    /*
     * 1. Turn on external clock
     * 2. Set CLKSEL = 1 and wait for wakeup
     * 3. Set PDWN and RESET = 1
     * 4. wait for VCAP >= 1.1V
     * 5. Reset pulse and wait for 18 tCLKs
     * 6. SDATAC
     * 7. External reference logic?
     * 8. Write certin registers
     * 9. Set start = 1
     * 10. Set to RDATAC
     * 11. Capture data and check noise
     * 12. Set test signals
     * 13. Caputure data and Test Signals
     */


    // 1. Analog and Digital Power-Up
    // (Assumes power rails are stable before calling this sequence)

    // 2. Clock Selection
    //if (use_external_clock) {
        // Set CLKSEL Pin = 0 and Provide External Clock fCLK = 2.048 MHz
        // Note: CLKSEL is typically a hardware pin. If tied to a GPIO, control it here.
        // e.g., gpio_set_level(dev->config.clksel_pin, 0);
    //} else {
        // Set CLKSEL Pin = 1 and Wait for Oscillator to Wake Up
        // e.g., gpio_set_level(dev->config.clksel_pin, 1);
        // If START pin is tied High, DRDY will toggle at fCLK / 8192 after this step
    //}

    // 3. Reset and Power Down Pin Setup
    // Set PDWN = 1 and RESET = 1
    // gpio_set_level(dev->config.pdwn_pin, 1);
    err = gpio_set_level(dev->config.reset_pin, 1);
    ESP_ERROR_CHECK(err);
    // 4. Wait for VCAP1 >= 1.1 V
    // If VCAP1 < 1.1 V at t_POR, continue waiting until VCAP1 >= 1.1 V
    // Note: Usually implemented as a fixed conservative safe delay (e.g., 500ms total)
    // unless VCAP1 is actively monitored by an ADC channel on the MCU.
    vTaskDelay(pdMS_TO_TICKS(350));

    // 5. Issue Reset Pulse, Wait for 18 tCLKs
    // Hardware reset pulse: pull low, wait (~2µs), pull high, wait 18 tCLKs (~9µs)
    err = ads1299_reset_hardware(dev);
    ESP_ERROR_CHECK(err);

    // 6. Send SDATAC Command
    // Device wakes up in RDATAC mode, so send SDATAC so registers can be written
    err = ads1299_disable_continuous_read(dev);
    ESP_ERROR_CHECK(err);

    // 7. External / Internal Reference Configuration
    // if (use_external_reference) {
    //     // No action required on PDB_REFBUF if using external reference
    // } else {
    //     // If Using Internal Reference, Send This Command: WREG CONFIG3 E0h
    //     err = ads1299_write_register(dev, ADS1299_REG_CONFIG3, 0xE0);
    //     if (err != ESP_OK) return err;
    //
    //     // Set PDB_REFBUF = 1 and Wait for Internal Reference to Settle
    //     // Internal reference settling time typically takes ~150ms
    //     vTaskDelay(pdMS_TO_TICKS(150));
    // }

    // 8. Write Certain Registers, Including Input Short
    // WREG CONFIG1 96h (Set Device for DR = fMOD / 4096)
    err = ads1299_write_register(dev, ADS1299_REG_CONFIG1, dev->config.sample_rate);
    ESP_ERROR_CHECK(err);

    // WREG CONFIG2 C0h
    err = ads1299_write_register(dev, ADS1299_REG_CONFIG2, 0xC0);
    ESP_ERROR_CHECK(err);

    // WREG CHnSET 01h (Set All Channels to Input Short)
    for (uint8_t i = 1; i <= 8; i++) {
        err = ads1299_write_register(dev, (ADS1299_REG_CH1SET + (i - 1)), 0x01);
        ESP_ERROR_CHECK(err);
    }

    // 9. Set START = 1
    // Activate Conversion. After this point DRDY toggles at fCLK / 8192
    err = ads1299_start(dev);
    ESP_ERROR_CHECK(err);

    // 10. RDATAC
    // Put the Device Back in RDATAC Mode
    err = ads1299_enable_continuous_read(dev);
    ESP_ERROR_CHECK(err);

    // 11. Capture Data and Check Noise
    // Look for DRDY and issue 24 + n x 24 SCLKs
    ads1299_sample_t noise_sample;
    // (In application logic, you would wait for the DRDY pin interrupt here)
    err = ads1299_read_sample(dev, &noise_sample);
    ESP_ERROR_CHECK(err);

    // 12. Set Test Signals
    // Activate a (1 mV x VREF / 2.4) Square-Wave Test Signal On All Channels

    // Step A: SDATAC (Must exit RDATAC to write registers)
    err = ads1299_disable_continuous_read(dev);
    ESP_ERROR_CHECK(err);

    // Step B: WREG CONFIG2 D0h
    err = ads1299_write_register(dev, ADS1299_REG_CONFIG2, 0xD0);
    ESP_ERROR_CHECK(err);

    // Step C: WREG CHnSET 05h (Set all channels to Test Signal)
    for (uint8_t i = 1; i <= 8; i++) {
        err = ads1299_write_register(dev, (ADS1299_REG_CH1SET + (i - 1)), 0x05);
        ESP_ERROR_CHECK(err);
    }

    // Step D: RDATAC
    err = ads1299_enable_continuous_read(dev);
    ESP_ERROR_CHECK(err);

    // 13. Capture Data and Test Signal
    // Look for DRDY and Issue 24 + n x 24 SCLKs
    ads1299_sample_t test_signal_sample;
    // (In application logic, wait for DRDY pin interrupt)
    err = ads1299_read_sample(dev, &test_signal_sample);
    ESP_ERROR_CHECK(err);


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

    // FIX 1: = {0} zeroes out the override_freq_hz garbage memory
    spi_transaction_t t = {0};

    // FIX 2: Using tx_data avoids DMA stack alignment issues entirely
    t.flags = SPI_TRANS_USE_TXDATA;
    t.length = 8;
    t.tx_data[0] = command;

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

    spi_transaction_t t = {0};

    t.length = ADS1299_FRAME_SIZE * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

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
    return gpio_set_level(dev->config.start_pin, 1);
}

esp_err_t ads1299_stop(ads1299_t* dev)
{
    gpio_set_level(dev->config.start_pin, 0);
    return ads1299_send_command(dev, ADS1299_CMD_STOP);
}

esp_err_t ads1299_reset_hardware(ads1299_t* dev)
{
    esp_err_t ret = gpio_set_level(dev->config.reset_pin, 0);
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
    esp_rom_delay_us(ADS1299_T_WAKEUP);
    return ret;
}

esp_err_t ads1299_enable_continuous_read(ads1299_t* dev)
{
    esp_err_t ret = ads1299_send_command(dev, ADS1299_CMD_RDATAC);
    esp_rom_delay_us(ADS1299_T_RDATAC);
    return ret;
}

esp_err_t ads1299_disable_continuous_read(ads1299_t* dev)
{
    esp_err_t ret = ads1299_send_command(dev, ADS1299_CMD_SDATAC);
    esp_rom_delay_us(ADS1299_T_SDATAC);
    return ret;
}

bool ads1299_is_running(const ads1299_t* dev)
{
    if (!dev) {
        return false;
    }
    return dev->dma_ctx != nullptr;
}

void ads1299_parse_frame(const uint8_t* raw, int64_t timestamp, ads1299_sample_t* out)
{
    out->status[0] = raw[0];
    out->status[1] = raw[1];
    out->status[2] = raw[2];

    for (int ch = 0; ch < 8; ch++) {
        int offset = 3 + ch * 3;

        // Temporary 24-bit value stored in a 32-bit int for sign extension
        int32_t val = ((int32_t)raw[offset] << 16) |
                      ((int32_t)raw[offset + 1] << 8)  |
                       (int32_t)raw[offset + 2];

        // Bit manipulation magic to properly fill the left most bits based on the sign
        out->channels[ch] = (val << 8) >> 8;
    }
    out->timestamp_us = timestamp;

}

uint32_t ads1299_sample_rate_to_hz(ads1299_sample_rate_t rate)
{
    switch (rate)
    {
        case ADS1299_DR_16kSPS: return 16000;
        case ADS1299_DR_8kSPS:  return 8000;
        case ADS1299_DR_4kSPS:  return 4000;
        case ADS1299_DR_2kSPS:  return 2000;
        case ADS1299_DR_1kSPS:  return 1000;
        case ADS1299_DR_500SPS: return 500;
        case ADS1299_DR_250SPS: return 250;
        default: return 0;
    }
}
