#ifndef ADS1299_H
#define ADS1299_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "ads1299_defs.h"


// Structs

/* ---------------- CONFIG ---------------- */

typedef struct {
    spi_host_device_t spi_host;

    gpio_num_t cs_pin;
    gpio_num_t mosi_pin;
    gpio_num_t miso_pin;
    gpio_num_t sclk_pin;

    gpio_num_t drdy_pin;
    gpio_num_t reset_pin;
    gpio_num_t start_pin;

   ads1299_sample_rate_t sample_rate;

    // ads1299_channel_config_t channel_config[8];
} ads1299_config_t;


/* ---------------- DEVICE HANDLE ---------------- */

typedef struct {
    /* user config */
    ads1299_config_t config;

    /* runtime state */
    spi_device_handle_t spi_handle;
    SemaphoreHandle_t mutex;

    bool initialized;

} ads1299_t;


typedef struct {
    uint8_t status[3];
    int32_t channels[8];
} ads1299_sample_t;



/* Device Lifecycle */
ads1299_t ads1299_create(const ads1299_config_t *cfg);
esp_err_t ads1299_init(ads1299_t *dev);
esp_err_t ads1299_deinit(ads1299_t *dev);


/* Register Access */

esp_err_t ads1299_write_register(
    ads1299_t *dev,
    uint8_t reg,
    uint8_t value);

esp_err_t ads1299_read_register(
    ads1299_t *dev,
    uint8_t reg,
    uint8_t *value);

esp_err_t ads1299_write_registers(
    ads1299_t *dev,
    uint8_t start_reg,
    const uint8_t *data,
    size_t count);

esp_err_t ads1299_read_registers(
    ads1299_t *dev,
    uint8_t start_reg,
    uint8_t *data,
    size_t count);


/* Generic Commands */

esp_err_t ads1299_send_command(
    ads1299_t *dev,
    uint8_t command);


/* Data Acquisition */

esp_err_t ads1299_read_data(
    ads1299_t *dev,
    uint8_t *buffer
    );

esp_err_t ads1299_read_sample(
    ads1299_t *dev,
    ads1299_sample_t *sample);


/* Device Control */

esp_err_t ads1299_start(ads1299_t *dev);
esp_err_t ads1299_stop(ads1299_t *dev);

esp_err_t ads1299_reset_hardware(ads1299_t *dev);
esp_err_t ads1299_reset_software(ads1299_t *dev);

esp_err_t ads1299_standby(ads1299_t *dev);
esp_err_t ads1299_wakeup(ads1299_t *dev);


/* Continuous Read Mode */

esp_err_t ads1299_enable_continuous_read(ads1299_t *dev);
esp_err_t ads1299_disable_continuous_read(ads1299_t *dev);

#ifdef __cplusplus
}
#endif

#endif