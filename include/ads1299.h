/**
 * @file ads1299.h
 * @brief Public API for the ADS1299 ESP-IDF driver.
 *
 * The driver controls a Texas Instruments ADS1299 analog front-end over SPI.
 * Applications own SPI bus initialization. The driver owns the ADS1299 SPI
 * device handle, GPIO setup, and continuous acquisition resources.
 */

#ifndef ADS1299_H
#define ADS1299_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "ads1299_defs.h"

/**
 * @brief One fully parsed sample from the ADS1299.
 *
 * Channel values are sign-extended 24-bit ADC values in LSB units.
 */
typedef struct {
    uint8_t status[ADS1299_STATUS_BYTES];        /**< Status bytes from the ADS1299 frame. */
    int32_t channels[ADS1299_NUM_CHANNELS];      /**< Sign-extended channel samples. */
    int64_t timestamp_us;                        /**< Timestamp captured at DRDY falling edge. */
} ads1299_sample_t;

/**
 * @brief A chunk of parsed ADS1299 samples delivered by the driver.
 *
 * The samples pointer references driver-managed storage and is valid only for
 * the duration of the chunk callback. Copy samples that must outlive the
 * callback.
 */
typedef struct {
    const ads1299_sample_t *samples;             /**< Pointer to the first sample in the chunk. */
    size_t n_samples;                            /**< Number of samples in the chunk. */
    int64_t first_timestamp_us;                  /**< Timestamp of the first sample. */
    int64_t last_timestamp_us;                   /**< Timestamp of the last sample. */
    int64_t dropped_count;                       /**< Total number of samples dropped by the driver. */
    int64_t overflow_count;                      /**< Total number of ring or DMA overflows. */
} ads1299_chunk_t;

/**
 * @brief Called when a full continuous-acquisition chunk is ready.
 *
 * The callback runs from the driver's handler task, not from ISR context. Keep
 * callback work short and hand samples to an application task if processing is
 * expensive.
 *
 * @param[in] chunk Completed chunk of parsed samples.
 * @param[in] ctx User context pointer from ads1299_continuous_config_t.
 */
typedef void (*ads1299_chunk_cb_t)(const ads1299_chunk_t *chunk, void *ctx);

/**
 * @brief Called when the continuous-acquisition task sees a recoverable error.
 *
 * @param[in] err ESP-IDF error code.
 * @param[in] ctx User context pointer from ads1299_continuous_config_t.
 */
typedef void (*ads1299_error_cb_t)(esp_err_t err, void *ctx);

/**
 * @brief Static device configuration.
 *
 * The application must initialize the SPI bus for spi_host before calling
 * ads1299_init(). The driver calls spi_bus_add_device() and
 * spi_bus_remove_device() internally.
 */
typedef struct {
    spi_host_device_t spi_host;                  /**< SPI host, usually SPI2_HOST or SPI3_HOST. */
    gpio_num_t cs_pin;                           /**< Chip select pin, active low. */
    gpio_num_t drdy_pin;                         /**< Data-ready pin, active low falling edge. */
    gpio_num_t reset_pin;                        /**< Hardware reset pin, active low. */
    gpio_num_t start_pin;                        /**< Conversion start pin. */
    ads1299_sample_rate_t sample_rate;           /**< Initial ADS1299 output data rate. */
} ads1299_config_t;

/**
 * @brief Continuous-acquisition configuration.
 *
 * Buffer sizing is derived from sample_rate and chunk_duration_ms. The caller
 * does not manage DMA or chunk storage directly.
 */
typedef struct {
    ads1299_chunk_cb_t on_chunk;                 /**< Required callback for completed chunks. */
    ads1299_error_cb_t on_error;                 /**< Optional callback for recoverable errors. */
    void *ctx;                                   /**< User context passed to callbacks. */
    uint32_t chunk_duration_ms;                  /**< Chunk length in milliseconds. */
    uint32_t ring_buffer_chunks;                 /**< Chunk slots; 0 selects ADS1299_RING_BUF_SLOTS. */
    UBaseType_t task_priority;                   /**< FreeRTOS priority for the handler task. */
    BaseType_t task_core;                        /**< FreeRTOS core affinity for the handler task. */
} ads1299_continuous_config_t;

/**
 * @brief Internal single-producer single-consumer chunk ring.
 *
 * This type is exposed only because the public handle is stack-allocatable.
 * Applications should not access it directly.
 */
typedef struct {
    ads1299_sample_t *buf;                       /**< Flat array of capacity * chunk_samples entries. */
    uint32_t capacity;                           /**< Total slot count; power of two and at least 2. */
    uint32_t chunk_samples;                      /**< Samples per chunk slot. */
    uint32_t mask;                               /**< capacity - 1, used for modulo indexing. */
    volatile uint32_t head;                      /**< Next write slot index. */
    volatile uint32_t tail;                      /**< Next read slot index. */
} ads1299_chunkring_t;

/**
 * @brief Opaque continuous-mode context.
 */
typedef struct ads1299_dma_ctx ads1299_dma_ctx_t;

/**
 * @brief ADS1299 device handle.
 *
 * Initialize with ads1299_create(), then ads1299_init(). Fields are public only
 * to allow stack allocation; applications should use the API functions for all
 * operations.
 */
typedef struct {
    ads1299_config_t config;                     /**< Static device configuration. */
    spi_device_handle_t spi_handle;              /**< ESP-IDF SPI device handle. */
    SemaphoreHandle_t mutex;                     /**< Driver mutex. */
    bool initialized;                            /**< True after successful initialization. */
    ads1299_dma_ctx_t *dma_ctx;                  /**< Non-NULL while continuous acquisition is active. */
} ads1299_t;

/**
 * @brief Create an ADS1299 device handle from static configuration.
 *
 * @param[in] cfg Pointer to the device configuration.
 * @return Initialized handle value. If cfg is NULL, the returned handle is
 *         zero-initialized and not ready for ads1299_init().
 */
ads1299_t ads1299_create(const ads1299_config_t *cfg);

/**
 * @brief Initialize the ADS1299 device.
 *
 * This configures GPIOs, adds the ADS1299 SPI device, creates the driver mutex,
 * and runs the power-up sequence. The SPI bus must already be initialized by
 * the application.
 *
 * @param[in,out] dev Device handle created by ads1299_create().
 * @return ESP_OK on success, or an ESP-IDF error code.
 */
esp_err_t ads1299_init(ads1299_t *dev);

/**
 * @brief Deinitialize the ADS1299 device and free driver-owned resources.
 *
 * @param[in,out] dev Device handle.
 * @return ESP_OK on success, or an ESP-IDF error code.
 */
esp_err_t ads1299_deinit(ads1299_t *dev);

/**
 * @brief Write one ADS1299 register.
 *
 * @param[in,out] dev Device handle.
 * @param[in] reg Register address.
 * @param[in] value Register value to write.
 * @return ESP_OK on success, or an ESP-IDF error code.
 */
esp_err_t ads1299_write_register(ads1299_t *dev, uint8_t reg, uint8_t value);

/**
 * @brief Read one ADS1299 register.
 *
 * @param[in,out] dev Device handle.
 * @param[in] reg Register address.
 * @param[out] value Destination for the register value.
 * @return ESP_OK on success, or an ESP-IDF error code.
 */
esp_err_t ads1299_read_register(ads1299_t *dev, uint8_t reg, uint8_t *value);

/**
 * @brief Write a contiguous ADS1299 register range.
 *
 * @param[in,out] dev Device handle.
 * @param[in] start_reg First register address.
 * @param[in] data Values to write.
 * @param[in] count Number of register values in data.
 * @return ESP_OK on success, or an ESP-IDF error code.
 */
esp_err_t ads1299_write_registers(ads1299_t *dev, uint8_t start_reg,
                                  const uint8_t *data, size_t count);

/**
 * @brief Read a contiguous ADS1299 register range.
 *
 * @param[in,out] dev Device handle.
 * @param[in] start_reg First register address.
 * @param[out] data Destination buffer.
 * @param[in] count Number of register values to read.
 * @return ESP_OK on success, or an ESP-IDF error code.
 */
esp_err_t ads1299_read_registers(ads1299_t *dev, uint8_t start_reg,
                                 uint8_t *data, size_t count);

/**
 * @brief Send a raw ADS1299 SPI command byte.
 *
 * Prefer named helper functions for common device-control commands.
 *
 * @param[in,out] dev Device handle.
 * @param[in] command ADS1299 command opcode.
 * @return ESP_OK on success, or an ESP-IDF error code.
 */
esp_err_t ads1299_send_command(ads1299_t *dev, uint8_t command);

/**
 * @brief Read one raw 27-byte ADS1299 frame.
 *
 * @param[in,out] dev Device handle.
 * @param[out] buffer Destination buffer of at least ADS1299_FRAME_SIZE bytes.
 * @return ESP_OK on success, or an ESP-IDF error code.
 */
esp_err_t ads1299_read_data(ads1299_t *dev, uint8_t *buffer);

/**
 * @brief Read and parse one ADS1299 sample.
 *
 * DRDY must already be low before this function is called.
 *
 * @param[in,out] dev Device handle.
 * @param[out] sample Parsed sample destination.
 * @return ESP_OK on success, or an ESP-IDF error code.
 */
esp_err_t ads1299_read_sample(ads1299_t *dev, ads1299_sample_t *sample);

/**
 * @brief Assert the START pin.
 *
 * @param[in,out] dev Device handle.
 * @return ESP_OK on success, or an ESP-IDF error code.
 */
esp_err_t ads1299_start(ads1299_t *dev);

/**
 * @brief Deassert the START pin.
 *
 * @param[in,out] dev Device handle.
 * @return ESP_OK on success, or an ESP-IDF error code.
 */
esp_err_t ads1299_stop(ads1299_t *dev);

/**
 * @brief Pulse the ADS1299 hardware RESET pin.
 *
 * @param[in,out] dev Device handle.
 * @return ESP_OK on success, or an ESP-IDF error code.
 */
esp_err_t ads1299_reset_hardware(ads1299_t *dev);

/**
 * @brief Send the ADS1299 RESET command.
 *
 * @param[in,out] dev Device handle.
 * @return ESP_OK on success, or an ESP-IDF error code.
 */
esp_err_t ads1299_reset_software(ads1299_t *dev);

/**
 * @brief Send the ADS1299 STANDBY command.
 *
 * @param[in,out] dev Device handle.
 * @return ESP_OK on success, or an ESP-IDF error code.
 */
esp_err_t ads1299_standby(ads1299_t *dev);

/**
 * @brief Send the ADS1299 WAKEUP command.
 *
 * @param[in,out] dev Device handle.
 * @return ESP_OK on success, or an ESP-IDF error code.
 */
esp_err_t ads1299_wakeup(ads1299_t *dev);

/**
 * @brief Enter ADS1299 continuous read mode.
 *
 * This sends RDATAC and is required before ads1299_start_continuous().
 *
 * @param[in,out] dev Device handle.
 * @return ESP_OK on success, or an ESP-IDF error code.
 */
esp_err_t ads1299_enable_continuous_read(ads1299_t *dev);

/**
 * @brief Exit ADS1299 continuous read mode.
 *
 * This sends SDATAC and is required before register writes.
 *
 * @param[in,out] dev Device handle.
 * @return ESP_OK on success, or an ESP-IDF error code.
 */
esp_err_t ads1299_disable_continuous_read(ads1299_t *dev);

/**
 * @brief Begin DMA-driven continuous acquisition.
 *
 * The driver allocates DMA buffers and chunk storage, installs the DRDY ISR,
 * starts a handler task, and calls on_chunk as chunks complete.
 *
 * @param[in,out] dev Device handle.
 * @param[in] cfg Continuous-acquisition configuration.
 * @return ESP_OK on success, or an ESP-IDF error code.
 */
esp_err_t ads1299_start_continuous(ads1299_t *dev,
                                   const ads1299_continuous_config_t *cfg);

/**
 * @brief Stop continuous acquisition and free acquisition resources.
 *
 * @param[in,out] dev Device handle.
 * @return ESP_OK on success, or an ESP-IDF error code.
 */
esp_err_t ads1299_stop_continuous(ads1299_t *dev);

/**
 * @brief Check whether continuous acquisition is active.
 *
 * @param[in] dev Device handle.
 * @return true if continuous acquisition is active, false otherwise.
 */
bool ads1299_is_running(const ads1299_t *dev);

/**
 * @brief Get the driver-owned FreeRTOS ring buffer handle.
 *
 * The returned handle is owned by the driver. Applications must not delete it.
 *
 * @param[in] dev Device handle.
 * @return Ring buffer handle, or NULL when continuous mode is inactive.
 */
RingbufHandle_t ads1299_get_ring_buffer(const ads1299_t *dev);

/**
 * @brief Parse a raw ADS1299 frame.
 *
 * @param[in] raw Raw ADS1299 frame of ADS1299_FRAME_SIZE bytes.
 * @param[in] timestamp Timestamp to store in the parsed sample.
 * @param[out] out Parsed sample destination.
 */
void ads1299_parse_frame(const uint8_t *raw, int64_t timestamp,
                         ads1299_sample_t *out);

/**
 * @brief Convert an ADS1299 sample-rate enum to hertz.
 *
 * @param[in] rate ADS1299 sample-rate enum value.
 * @return Sample rate in hertz, or 0 for an invalid value.
 */
uint32_t ads1299_sample_rate_to_hz(ads1299_sample_rate_t rate);

#ifdef __cplusplus
}
#endif

#endif /* ADS1299_H */
