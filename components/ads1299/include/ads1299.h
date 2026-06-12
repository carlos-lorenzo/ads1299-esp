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

/* ================================================================
 * CONSTANTS
 * ================================================================ */

#define ADS1299_NUM_CHANNELS      8
#define ADS1299_STATUS_BYTES      3
#define ADS1299_BYTES_PER_CHANNEL 3
#define ADS1299_FRAME_BYTES \
    (ADS1299_STATUS_BYTES + ADS1299_NUM_CHANNELS * ADS1299_BYTES_PER_CHANNEL) // 27

/** Default DMA chunk duration in milliseconds.
 *  At 250SPS this yields 25 samples per chunk (675 bytes per DMA buffer).
 *  Override via ads1299_continuous_config_t.chunk_duration_ms. */
#define ADS1299_DEFAULT_CHUNK_MS  100

/** Minimum ring buffer slots. Driver allocates
 *  ADS1299_RING_BUF_SLOTS * chunk_bytes of memory at start_continuous(). */
#define ADS1299_RING_BUF_SLOTS    8

/* ================================================================
 * DATA TYPES
 * ================================================================ */

/**
 * One fully parsed sample from the ADS1299.
 *
 - channels[] contain sign-extended 24-bit values in LSB units.
 - Convert to microvolts: uV = channels[n] * (VREF / gain / 8388607.0)
 - where VREF = 4.5V for ADS1299, and gain is set per channel in CHnSET.
 */
typedef struct {
    uint8_t  status[ADS1299_STATUS_BYTES]; // GPIO, lead-off, lock bits
    int32_t  channels[ADS1299_NUM_CHANNELS];
    int64_t  timestamp_us;                 // esp_timer_get_time() at DRDY fall
} ads1299_sample_t;

/**
 - One chunk — chunk_samples parsed samples delivered per callback.
 - The samples pointer is into the driver-managed ring buffer and is
 * valid only for the duration of on_chunk(). Copy if needed beyond that.
 */
typedef struct {
    const ads1299_sample_t *samples;
    size_t                  n_samples;
    int64_t                 first_timestamp_us;
    int64_t                 last_timestamp_us;
} ads1299_chunk_t;

/* ================================================================
 * CALLBACKS
 * ================================================================ */

/**
 * Called from the driver's handler task (NOT ISR context) each time a
 * full chunk is ready. chunk->samples is valid only during this call.
 *
 - Execution context: driver handler task on task_core at task_priority.
 - Keep this short — hand off to your processing task via a queue or
 * ring buffer rather than filtering or running inference here.
 *
 * @param chunk   Completed chunk of parsed samples
 * @param ctx     User context pointer from ads1299_continuous_config_t
 */
typedef void (*ads1299_chunk_cb_t)(const ads1299_chunk_t *chunk, void *ctx);

/**
 * Called from the driver's handler task when a recoverable error occurs
 * (DMA overrun, SPI transaction failure, ring buffer full).
 *
 * @param err     ESP-IDF error code
 * @param ctx     User context pointer from ads1299_continuous_config_t
 */
typedef void (*ads1299_error_cb_t)(esp_err_t err, void *ctx);

/* ================================================================
 * CONFIGURATION
 * ================================================================ */

/**
 * Static device configuration. Provided once at ads1299_create().
 * The SPI bus (spi_bus_initialize) must be initialised by the caller
 * before ads1299_init() is called. The driver only calls
 * spi_bus_add_device() / spi_bus_remove_device() internally.
 */
typedef struct {
    spi_host_device_t     spi_host;    // SPI2_HOST or SPI3_HOST
    gpio_num_t            cs_pin;      // chip select (active low)
    gpio_num_t            drdy_pin;    // data ready (active low, falling edge)
    gpio_num_t            reset_pin;   // hardware reset (active low)
    gpio_num_t            start_pin;   // conversion start
    ads1299_sample_rate_t sample_rate; // ADS1299_DR_250SPS .. ADS1299_DR_16KSPS
} ads1299_config_t;

/**
 * Continuous acquisition configuration. Provided to ads1299_start_continuous().
 * All buffer sizing is derived automatically from config.sample_rate and
 * chunk_duration_ms — the caller does not manage any buffers directly.
 */
typedef struct {
    /** Required. Called with each completed chunk. See ads1299_chunk_cb_t. */
    ads1299_chunk_cb_t on_chunk;

    /** Optional. Called on driver errors. NULL to silently discard errors. */
    ads1299_error_cb_t on_error;

    /** Passed back unmodified in both callbacks. */
    void *ctx;

    /** Chunk duration in milliseconds. 0 = ADS1299_DEFAULT_CHUNK_MS.
     *  Determines DMA buffer size: chunk_samples = sample_rate_hz * ms / 1000.
     *  Must divide evenly into the sample rate for accurate timing. */
    uint32_t chunk_duration_ms;

    /** FreeRTOS priority for the driver's internal handler task.
     *  Should be higher than your processing task to avoid starving the ISR.
     *  Recommended: configMAX_PRIORITIES - 2 */
    UBaseType_t task_priority;

    /** Core affinity for the handler task. 0 or 1. Use tskNO_AFFINITY to
     *  let the scheduler decide. Recommended: 1 (leave core 0 for networking) */
    BaseType_t task_core;
} ads1299_continuous_config_t;

/* ================================================================
 * DEVICE HANDLE
 * ================================================================ */

/** Opaque continuous-mode context. Defined only in ads1299.c. */
typedef struct ads1299_dma_ctx ads1299_dma_ctx_t;

/**
 * Device handle. Initialise with ads1299_create(), then ads1299_init().
 * Do not access fields directly — they are public only to allow
 * stack allocation. Use the API functions for all operations.
 */
typedef struct {
    ads1299_config_t    config;
    spi_device_handle_t spi_handle;
    SemaphoreHandle_t   mutex;
    bool                initialized;

    /** Non-NULL only while ads1299_start_continuous() is active. */
    ads1299_dma_ctx_t  *dma_ctx;
} ads1299_t;

/* ================================================================
 * INTERNAL DMA CONTEXT  (defined in ads1299.c, never exposed)
 *
 * Documented here for contributor reference only.
 *
 * struct ads1299_dma_ctx {
 *     uint8_t        *ping;            // DMA-capable buffer A
 *     uint8_t        *pong;            // DMA-capable buffer B
 *     uint8_t        *active;          // points to ping or pong
 *     size_t          chunk_samples;   // samples per chunk
 *     size_t          buf_bytes;       // chunk_samples * ADS1299_FRAME_BYTES
 *     size_t          sample_count;    // samples written into active buffer
 *     RingbufHandle_t ring;            // parsed chunks → processing task
 *     TaskHandle_t    handler_task;    // woken by TaskNotify from ISR
 *     ads1299_chunk_cb_t  on_chunk;
 *     ads1299_error_cb_t  on_error;
 *     void               *ctx;
 * };
 *
 * ISR flow:
 *   DRDY falls
 *     → SPI DMA transaction queued (27 bytes into active[sample_count])
 *     → SPI post_cb: sample_count++
 *     → if sample_count == chunk_samples:
 *         swap active buffer (ping↔pong)
 *         reset sample_count
 *         xTaskNotifyFromISR(handler_task, ready_buf_index, eSetValueWithOverwrite)
 *
 * Handler task flow:
 *   xTaskNotifyWait()
 *     → parse raw bytes → ads1299_sample_t[]
 *     → fill ads1299_chunk_t
 *     → on_chunk(&chunk, ctx)
 *     → xRingbufferSend(ring, &chunk, ...)   // optional if app uses ring directly
 * ================================================================ */

/* ================================================================
 * LIFECYCLE
 * ================================================================ */

/**
 * Zero-initialise a handle and copy config into it. No hardware interaction.
 * Call before ads1299_init().
 */
ads1299_t ads1299_create(const ads1299_config_t *cfg);

/**
 * Initialise hardware: configure GPIO outputs, add SPI device, create mutex,
 * run the ADS1299 power-up sequence (reset, SDATAC, register config, test signal).
 *
 * Prerequisites:
 *   - spi_bus_initialize() called by the application for cfg->spi_host
 *   - Power rails stable, VCAP1 will reach 1.1V within ~350ms
 *
 * @return ESP_OK on success
 *         ESP_ERR_INVALID_ARG if dev is NULL
 *         ESP_ERR_INVALID_STATE if already initialised
 *         ESP_ERR_NO_MEM if mutex allocation fails
 *         propagated SPI / GPIO errors
 */
esp_err_t ads1299_init(ads1299_t *dev);

/**
 * Stop any active continuous acquisition, remove SPI device, delete mutex,
 * free all driver-owned memory. Safe to call from any task context.
 */
esp_err_t ads1299_deinit(ads1299_t *dev);

/* ================================================================
 * REGISTER ACCESS
 * ================================================================ */

/** Write a single register. Not valid while continuous mode is active
 *  (device must be in SDATAC). */
esp_err_t ads1299_write_register(ads1299_t *dev, uint8_t reg, uint8_t value);
esp_err_t ads1299_read_register(ads1299_t *dev, uint8_t reg, uint8_t *value);

/** Burst register access — wraps WREG/RREG with count. */
esp_err_t ads1299_write_registers(ads1299_t *dev, uint8_t start_reg,
                                   const uint8_t *data, size_t count);
esp_err_t ads1299_read_registers(ads1299_t *dev, uint8_t start_reg,
                                  uint8_t *data, size_t count);

/* ================================================================
 * COMMANDS
 * ================================================================ */

/** Send a raw SPI command byte (WAKEUP, STANDBY, RESET, START, STOP,
 *  RDATAC, SDATAC, RDATA). Use named wrappers below where possible. */
esp_err_t ads1299_send_command(ads1299_t *dev, uint8_t command);

/* ================================================================
 * ONE-SHOT DATA READ
 * Only valid when NOT in continuous mode. Blocks until SPI completes.
 * ================================================================ */

/** Read raw 27-byte frame into caller-provided buffer. */
esp_err_t ads1299_read_data(ads1299_t *dev, uint8_t *buffer);

/** Read and parse one sample. DRDY must already be low. */
esp_err_t ads1299_read_sample(ads1299_t *dev, ads1299_sample_t *sample);

/* ================================================================
 * DEVICE CONTROL
 * ================================================================ */

esp_err_t ads1299_start(ads1299_t *dev);           // assert START pin
esp_err_t ads1299_stop(ads1299_t *dev);            // deassert START pin
esp_err_t ads1299_reset_hardware(ads1299_t *dev);  // pulse RESET pin
esp_err_t ads1299_reset_software(ads1299_t *dev);  // send RESET command
esp_err_t ads1299_standby(ads1299_t *dev);         // send STANDBY command
esp_err_t ads1299_wakeup(ads1299_t *dev);          // send WAKEUP command

/* ================================================================
 * RDATAC / SDATAC
 * ================================================================ */

/** Enter continuous read mode (RDATAC). Required before ads1299_start_continuous(). */
esp_err_t ads1299_enable_continuous_read(ads1299_t *dev);

/** Exit continuous read mode (SDATAC). Required before register writes. */
esp_err_t ads1299_disable_continuous_read(ads1299_t *dev);

/* ================================================================
 * CONTINUOUS ACQUISITION  (DMA + TaskNotify + ring buffer)
 * ================================================================ */

/**
 * Begin DMA-driven continuous acquisition.
 *
 * Internally:
 *   1. Computes chunk_samples = sample_rate_hz * chunk_duration_ms / 1000
 *   2. Allocates two DMA-capable ping-pong buffers of chunk_samples * 27 bytes
 *   3. Allocates ring buffer of ADS1299_RING_BUF_SLOTS * chunk_bytes
 *   4. Installs falling-edge ISR on drdy_pin
 *   5. Spawns handler task (task_core, task_priority)
 *   6. ISR → xTaskNotifyFromISR → handler task → on_chunk callback
 *
 * ads1299_enable_continuous_read() must be called before this.
 * Register writes are not permitted while acquisition is active.
 *
 * @return ESP_ERR_INVALID_STATE  if not initialised or already running
 *         ESP_ERR_INVALID_ARG    if on_chunk is NULL
 *         ESP_ERR_NO_MEM         if buffer allocation fails
 */
esp_err_t ads1299_start_continuous(ads1299_t *dev,
                                    const ads1299_continuous_config_t *cfg);

/**
 * Stop acquisition cleanly:
 *   - Disables DRDY ISR
 *   - Signals handler task to exit and waits for it
 *   - Frees ping-pong DMA buffers and ring buffer
 *   - Sets dma_ctx to NULL
 *
 * Blocks until the handler task has exited. Safe to call from any task.
 */
esp_err_t ads1299_stop_continuous(ads1299_t *dev);

/** @return true if ads1299_start_continuous() is active */
bool ads1299_is_running(const ads1299_t *dev);

/**
 * Optional: expose the ring buffer handle for applications that want to
 * consume chunks directly via xRingbufferReceive() rather than the callback.
 * Returns NULL if continuous mode is not active.
 *
 * Ownership: the ring buffer is owned by the driver. Do not delete it.
 * Call xRingbufferReturnItem() after processing each received item.
 */
RingbufHandle_t ads1299_get_ring_buffer(const ads1299_t *dev);

/* ================================================================
 * UTILITY
 * ================================================================ */

/**
 * Parse a raw 27-byte ADS1299 frame into an ads1299_sample_t.
 * Exposed for unit testing. Not needed in normal application code.
 *
 * @param raw       27-byte frame: 3 status + 8 * 3 channel bytes (big-endian)
 * @param timestamp esp_timer_get_time() value to embed in the sample
 * @param out       destination sample struct
 */
void ads1299_parse_frame(const uint8_t *raw, int64_t timestamp,
                          ads1299_sample_t *out);

/**
 * Return the sample rate in Hz for a given ads1299_sample_rate_t enum value.
 * Useful for computing chunk sizes and window dimensions in application code.
 */
uint32_t ads1299_sample_rate_to_hz(ads1299_sample_rate_t rate);

#ifdef __cplusplus
}
#endif

#endif /* ADS1299_H */