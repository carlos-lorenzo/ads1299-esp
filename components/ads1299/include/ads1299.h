#ifndef ADS1299_H
#define ADS1299_H

#ifdef __cplusplus
extern "C" {
#endif



#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "ads1299_defs.h"


typedef struct {
    bool enabled; 
    ads1299_pga_gain_t gain;                      
    ads1299_input_mux_t input_mux; 
} ads1299_channel_config_t;

typedef struct {
    spi_host_device_t spi_host;  // SPI host (struct from the driver library to specify which SPI peripheral to use)
    
    gpio_num_t cs_pin;                  // Chip Select GPIO pin
    gpio_num_t mosi_pin;                // Master Out Slave In GPIO pin
    gpio_num_t miso_pin;                // Master In Slave Out GPIO pin
    gpio_num_t sclk_pin;                // Serial Clock GPIO pin

    gpio_num_t drdy_pin;                // Data Ready GPIO pin
    gpio_num_t reset_pin;               // Reset GPIO pin
    gpio_num_t start_pin;               // Start GPIO pin

    ads1299_sample_rate_t sample_rate;      // Data output rate (e.g., 250 SPS, 500 SPS, etc.)
    // ads1299_channel_config_t channel_configs[8]; // Configuration for each of the 8 channels

   
} ads1299_config_t;


/*
Initialize the ADS1299 with the specified configuration. This function will set up the SPI interface, configure the GPIO pins, and perform any necessary initialization steps to prepare the ADS1299 for operation.
@params config: Pointer to the configuration structure

returns:
errors
*/
esp_err_t ads1299_init(const ads1299_config_t* config);

// Generic SPI read/write functions for the ADS1299. These will be used by the higher-level functions to send commands and read/write registers. They will handle the SPI transactions, including asserting the CS pin, sending the command byte(s), and reading/writing data as needed.
esp_err_t write_register(ads1299_config_t* config, uint8_t reg, uint8_t value);
esp_err_t read_register(ads1299_config_t* config, uint8_t reg, uint8_t* value);

esp_err_t send_command(ads1299_config_t* config, uint8_t command);
esp_err_t send_command_with_data(ads1299_config_t* config, uint8_t command, const uint8_t* data, size_t data_len);

// Read data will work by an interrupt on the DRDY pin, which will trigger a read of the data registers when new data is available. It will use DMA to transfer the data from the ADS1299 to a buffer in memory
esp_err_t read_data(ads1299_config_t* config, uint8_t* buffer, size_t buffer_len);


// Specific commands
esp_err_t ads1299_start_hardware(const ads1299_config_t* config);
esp_err_t ads1299_start_software(const ads1299_config_t* config);

esp_err_t ads1299_stop(const ads1299_config_t* config);

esp_err_t ads1299_reset_hardware(const ads1299_config_t* config);
esp_err_t ads1299_reset_software(const ads1299_config_t* config);

esp_err_t ads1299_standby(const ads1299_config_t* config);
esp_err_t ads1299_wakeup(const ads1299_config_t* config);


void do_something(void);

#ifdef __cplusplus
}
#endif

#endif