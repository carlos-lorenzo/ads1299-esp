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
    int64_t dropped_count;
    int64_t overflow_count;
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



/**
 * @brief Driver configuration structure.
 */
typedef struct {
    spi_host_device_t     spi_host;    // SPI2_HOST or SPI3_HOST
    gpio_num_t            cs_pin;      // chip select (active low)
    gpio_num_t            drdy_pin;    // data ready (active low, falling edge)
    gpio_num_t            reset_pin;   // hardware reset (active low)
    gpio_num_t            start_pin;   // conversion start
    ads1299_sample_rate_t sample_rate; // ADS1299_DR_250SPS - ADS1299_DR_16KSPS
    } ads1299_config_t;

/**
 * Continuous acquisition configuration. Provided to ads1299_start_continuous().
 * All buffer sizing is derived automatically from config.sample_rate and
 * chunk_duration_ms — the caller does not manage any buffers directly.
 */
typedef struct {
    ads1299_chunk_cb_t on_chunk;
    ads1299_error_cb_t on_error;
    void              *ctx;
    uint32_t           chunk_duration_ms;

    /**
     * Ring buffer depth in number of chunk slots.
     * 0 = ADS1299_RING_BUF_SLOTS (default).
     *
     * Must be a power of 2. Rounded up internally if not.
     *
     * Total memory allocated:
     *   ring_buffer_chunks * chunk_samples * sizeof(ads1299_sample_t)
     *
     * At 250 SPS, 100 ms chunks, 8 slots:
     *   8 * 25 * 44 = 8,800 bytes
     *
     * At 16 kSPS, 10 ms chunks, 8 slots:
     *   8 * 160 * 44 = 56,320 bytes
     *
     * At 16 kSPS, 100 ms chunks, 8 slots:
     *   8 * 1600 * 44 = 563,200 bytes  ← EXCEEDS ESP32 SRAM; reduce chunk_duration_ms
     */
    uint32_t    ring_buffer_chunks;

    UBaseType_t task_priority;
    BaseType_t  task_core;
} ads1299_continuous_config_t;



/*
 * Circular buffer of fully parsed ADS1299 chunks.
 *
 * Layout of buf[]:
 *
 *   slot 0                    slot 1
 *   [s0|s1|...|sN-1]          [s0|s1|...|sN-1]    ...
 *   ^                          ^
 *   buf[0]                     buf[chunk_samples]
 *
 * A slot is addressed as: buf[(index & mask) * chunk_samples + sample_i]
 *
 * capacity slots are physically allocated.
 * capacity-1 slots are usable (one sentinel slot disambiguates full from empty).
 *
 * SPSC ownership:
 *   head — written exclusively by ISR (post_cb)
 *   tail — written exclusively by handler task
 */
typedef struct {
    ads1299_sample_t *buf;           /* flat array: capacity * chunk_samples entries  */
    uint32_t          capacity;      /* total slot count; must be power-of-2, >= 2   */
    uint32_t          chunk_samples; /* samples per slot                              */
    uint32_t          mask;          /* capacity - 1; used for slot index modulo      */
    volatile uint32_t head;          /* next write slot index; ISR-owned             */
    volatile uint32_t tail;          /* next read slot index; task-owned             */
} ads1299_chunkring_t;


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
 * LIFECYCLE
 * ================================================================ */

/**
 * @brief Creates and initialises an ADS1299 device handle.
 *
 *
 * @param[in] cfg Pointer to configuration structure containing pin maps.
 * @return
 *     - ads1299_t: Object
 *     - Null: Invalid arguments
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



 /**
 * @brief Initialize the ADS1299 driver.
 *
 * @param[in] config Pointer to driver configuration.
 * @return ESP_OK on success, or an error code.
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