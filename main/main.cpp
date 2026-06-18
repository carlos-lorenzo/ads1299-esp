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


void on_chunk(const ads1299_chunk_t *chunk, void *ctx)
{
    ESP_LOGI("ADS1299", "Received chunk with %d samples", chunk->n_samples);
}


extern "C" void app_main(void)
{
    spi_bus_config_t bus_cfg = {};
    bus_cfg.miso_io_num = MISO_PIN;
    bus_cfg.mosi_io_num = MOSI_PIN;
    bus_cfg.sclk_io_num = SCLK_PIN;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = ADS1299_FRAME_BYTES * 25;


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
    cont_cfg.task_priority = configMAX_PRIORITIES - 2;
    cont_cfg.task_core = 0;
    ESP_ERROR_CHECK(ads1299_start_continuous(&dev1, &cont_cfg));


    // 0 initialized raw bytestream
    uint8_t raw[ADS1299_FRAME_BYTES] = {0x00};
    // int64_t timestamp = esp_timer_get_time();
    // ads1299_sample_t sample;
    // ads1299_parse_frame(raw, timestamp, &sample);
    // // Show sample parsed at timestep: <time>\n with status <status bytes>\n Channels:\n each channel's valESP_LOGI(TAG, "Handler task called"); // TODO: Remove this call as its blockingue
    // printf("Sample parsed at timestep: %lld us\n", sample.timestamp_us);
    // printf("Status: %02X %02X %02X\n", sample.status[0], sample.status[1], sample.status[2]);
    // for (int i = 0; i < ADS1299_NUM_CHANNELS; i++) {
    //     printf("Channel %d: %d\n", i + 1, static_cast<int>(sample.channels[i]));
    // }

    // 100 ms and 250SPS => 25 samples
    for (int i = 0; i < 25; i++)
    {
        gpio_set_level(DRDY_PIN, 0);
        vTaskDelay(pdTICKS_TO_MS(4)); // 1000/250 = 4
        gpio_set_level(DRDY_PIN, 1);
    }

}

