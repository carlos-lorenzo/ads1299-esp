#include "ads1299.h"
#include "ads1299_defs.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"


#define DRDY_PIN GPIO_NUM_8
#define MISO_PIN GPIO_NUM_9
#define SCLK_PIN GPIO_NUM_10
#define CS1_PIN GPIO_NUM_11
#define START_PIN GPIO_NUM_12
#define RESET_PIN GPIO_NUM_13
#define MOSI_PIN GPIO_NUM_14
#define CS2_PIN GPIO_NUM_21


static const char *TAG = "ADS1299";


void on_chunk(const ads1299_chunk_t *chunk, void *ctx)
{

    ESP_LOGI(TAG, "Received chunk with %zu samples between %lld and %lld whilst having dropped %d and overflown %d", chunk->n_samples, chunk->first_timestamp_us, chunk->last_timestamp_us, chunk->dropped_count, chunk->overflow_count);
    // for (int i = 0; i < chunk->n_samples; i++)
    // {
    //     const ads1299_sample_t *sample = &chunk->samples[i];
    //     ESP_LOGI(TAG, "Sample %d: Timestamp: %lld, Status: %02X %02X %02X, Channels: %d %d %d %d %d %d %d %d",
    //              i,
    //              sample->timestamp_us,
    //              sample->status[0], sample->status[1], sample->status[2],
    //              sample->channels[0], sample->channels[1], sample->channels[2], sample->channels[3],
    //              sample->channels[4], sample->channels[5], sample->channels[6], sample->channels[7]);
    // }
}


// 1. Define a clean, high-precision timer callback
static void emulated_drdy_timer_callback(void* arg)
{
    // High-precision pulse emulation matching real hardware behavior
    gpio_set_level(DRDY_PIN, 0);
    esp_rom_delay_us(10);
    gpio_set_level(DRDY_PIN, 1);
}

extern "C" void app_main(void)
{
    spi_bus_config_t bus_cfg = {};
    bus_cfg.miso_io_num = MISO_PIN;
    bus_cfg.mosi_io_num = MOSI_PIN;
    bus_cfg.sclk_io_num = SCLK_PIN;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = ADS1299_FRAME_SIZE * 25;


    ESP_ERROR_CHECK(
        spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO)
    );

    ads1299_config_t cfg1  = {};
    cfg1.spi_host = SPI2_HOST;
    cfg1.cs_pin = CS1_PIN;
    cfg1.drdy_pin = DRDY_PIN;
    cfg1.start_pin = START_PIN;
    cfg1.reset_pin = RESET_PIN;
    cfg1.sample_rate = ADS1299_DR_250SPS;
    ads1299_t dev1 = ads1299_create(&cfg1);

    ESP_ERROR_CHECK(ads1299_init(&dev1));

    // Run some configs...

    // Ensure ads1299 can transmit data by sending RDATAC command
    ads1299_enable_continuous_read(&dev1);

    ads1299_continuous_config_t cont_cfg = {};
    cont_cfg.on_chunk = on_chunk;
    cont_cfg.chunk_duration_ms = ADS1299_DEFAULT_CHUNK_MS; // 100 ms chunks
    cont_cfg.ring_buffer_chunks = ADS1299_RING_BUF_SLOTS; // 8 chunks in ring buffer
    cont_cfg.task_priority = configMAX_PRIORITIES - 2;
    cont_cfg.task_core = 0;
    ESP_ERROR_CHECK(ads1299_start_continuous(&dev1, &cont_cfg));



    // 2. Configure a dedicated microsecond-resolution hardware timer
    esp_timer_create_args_t periodic_timer_args = {}; // Zero-initialize everything first
    periodic_timer_args.callback = &emulated_drdy_timer_callback;
    periodic_timer_args.name = "emulated_drdy";

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));

    // 3. Start the timer at exactly 4000 microseconds (250 Hz)
    // This runs completely independently of FreeRTOS task slicing
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 4000));

    // Fall into standard execution loop
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

