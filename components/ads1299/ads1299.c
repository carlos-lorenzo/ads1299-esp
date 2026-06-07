#include <stdio.h>
#include "ads1299.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"




esp_err_t ads1299_reset_hardware(const ads1299_config_t* config)
{
    gpio_set_level(config->reset_pin, 0);
    esp_rom_delay_us(ADS1299_T_RESET);
    gpio_set_level(config->reset_pin, 1);

    return ESP_OK;
}

esp_err_t ads1299_reset_software(const ads1299_config_t* config)
{
    esp_err_t ret = send_command(config, ADS1299_CMD_RESET);
    esp_rom_delay_us(ADS1299_T_RESET);
    return ret;
}

esp_err_t ads1299_sdatac(const ads1299_config_t* config)
{
    esp_err_t ret = send_command(config, ADS1299_CMD_RESET);
    esp_rom_delay_us(ADS1299_T_SDATAC);
    return ret;
}

esp_err_t ads1299_init(ads1299_config_t *config)
{


    // Debug info
    printf("Initializing ADS1299 with the following configuration:\n");
    printf("Sample Rate: %d\n", config->sample_rate);
    // printf("Channel Configurations:\n");
    // for (int i = 0; i < 8; i++) {
    // printf("Channel %d: Enabled=%d, Gain=%d, Input=%d\n", i +1, config->channel_configs[i].enabled, config->channel_configs[i].gain, config->channel_configs[i].input_mux);
    // }

    // Gpio dump
    // gpio_dump_io_configuration(stdout, SOC_GPIO_VALID_GPIO_MASK);

    esp_err_t ret = ESP_OK;

    // Configure GPIO pins
    gpio_config_t pin_config = {
        .pin_bit_mask = (1ULL << config->cs_pin) | (1ULL << config->reset_pin) | (1ULL << config->start_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ret = gpio_config(&pin_config);
    ESP_ERROR_CHECK(ret);


    // Initialize SPI interface

    spi_bus_config_t bus_config = {
        .mosi_io_num = config->mosi_pin,
        .miso_io_num = config->miso_pin,
        .sclk_io_num = config->sclk_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0 // Can be set to 27 * n_daisy_chain devices if running with daisy chain (rounded up to a power of 2). Sets the size of the dma buffer
    };

    ret = spi_bus_initialize(config->spi_host, &bus_config, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t device_config = {
        .mode = 1,                                    // Mode 1 (CPOL=0, CPHA=1)
        .clock_speed_hz = 4 * 1000 * 1000,            // 4 MHz
        .spics_io_num = -1,                           // -1 = Manually managed via GPIO *CRITICAL*
        .queue_size = 3,                              // Small queue since transactions are sequential
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .flags = 0,
        .pre_cb = nullptr,
        .post_cb = nullptr
    };
    spi_bus_add_device(config->spi_host, &device_config, &config->spi_handle);


    ESP_ERROR_CHECK(ret);
    
    // Perform any necessary initialization steps for the ADS1299

    // 1. Set CLKSEL to 1 if not connected to VDD (set option in config) else set to 0 and init external clock
    // 2. Set PDWN to 1 if not connected to VDD (set option in config)
    // gpio_set_level(config->pdwn_pin, 1);
    // 3. Set RESET to 1
    // 4. Wait for VCAP1 > 1.1V
    // 5. Reset pulse and wait for 18 tclk
    // 6. Send SDATAC Command
    // 7. External reference config: // If Using Internal Reference, Send This Command WREG CONFIG3 E0h
    // or Set PDB_REFBUF = 1 and Wait for Internal Reference to Settle

    gpio_set_level(config->reset_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(2500));
    ads1299_reset_hardware(config);



    return ret;
}

esp_err_t send_command(const ads1299_config_t* config, uint8_t command)
{
    // Assert CS pin
    gpio_set_level(config->cs_pin, 0);
    esp_rom_delay_us(ADS1299_T_CSSC);

    // Send command byte over SPI
    spi_transaction_t transaction = {
        .length = 8, // Command is 1 byte (8 bits)
        .tx_buffer = &command,
        .rx_buffer = NULL
    };

    esp_err_t ret = spi_device_polling_transmit(config->spi_handle, &transaction);

    esp_rom_delay_us(ADS1299_T_CSH);

    // Deassert CS pin
    gpio_set_level(config->cs_pin, 1);

    return ret;
}
