/**
 * @file ads1299_defs.h
 * @brief Public constants and enums for the ADS1299 ESP-IDF driver.
 */

#ifndef ADS1299_DEFS_H
#define ADS1299_DEFS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** ADS1299 internal clock frequency in hertz. */
#define ADS1299_CLK_FREQ 2048000UL

/** Convert ADS1299 clock ticks to microseconds, rounded up. */
#define ADS1299_TICKS_TO_US(ticks) (((ticks) * 1000000UL + (ADS1299_CLK_FREQ - 1)) / ADS1299_CLK_FREQ)

/** One ADS1299 clock period in microseconds. */
#define ADS1299_T_CLK_US ADS1299_TICKS_TO_US(1)
/** Hardware reset pulse timing in microseconds. */
#define ADS1299_T_RESET ADS1299_TICKS_TO_US(18)
/** START or RESET pulse timing in microseconds. */
#define ADS1299_T_PULSE ADS1299_TICKS_TO_US(2)
/** SDATAC command timing in microseconds. */
#define ADS1299_T_SDATAC ADS1299_TICKS_TO_US(2)
/** RDATAC command timing in microseconds. */
#define ADS1299_T_RDATAC ADS1299_TICKS_TO_US(2)
/** WAKEUP command timing in microseconds. */
#define ADS1299_T_WAKEUP ADS1299_TICKS_TO_US(4)
/** Chip-select setup timing in microseconds. */
#define ADS1299_T_CSSC ADS1299_TICKS_TO_US(1)
/** SCLK timing in microseconds. */
#define ADS1299_T_SCLK ADS1299_TICKS_TO_US(1)
/** Chip-select hold timing in microseconds. */
#define ADS1299_T_CSH ADS1299_TICKS_TO_US(2)
/** SCLK-to-chip-select timing in microseconds. */
#define ADS1299_T_SCCS ADS1299_TICKS_TO_US(4)
/** Command decode timing in microseconds. */
#define ADS1299_T_SDECODE ADS1299_TICKS_TO_US(4)

/** Number of ADS1299 channels. */
#define ADS1299_NUM_CHANNELS 8
/** Number of status bytes at the start of each ADS1299 frame. */
#define ADS1299_STATUS_BYTES 3
/** Number of bytes per ADS1299 channel sample. */
#define ADS1299_BYTES_PER_CHANNEL 3
/** Total ADS1299 frame size in bytes. */
#define ADS1299_FRAME_SIZE (ADS1299_STATUS_BYTES + ADS1299_NUM_CHANNELS * ADS1299_BYTES_PER_CHANNEL)

/**
 * @brief Default DMA chunk duration in milliseconds.
 *
 * At 250 SPS this yields 25 samples per chunk. Override with
 * ads1299_continuous_config_t::chunk_duration_ms.
 */
#define ADS1299_DEFAULT_CHUNK_MS 100

/** Default number of chunk slots allocated for continuous acquisition. */
#define ADS1299_RING_BUF_SLOTS 8

/** ADS1299 WAKEUP command opcode. */
#define ADS1299_CMD_WAKEUP 0x02
/** ADS1299 STANDBY command opcode. */
#define ADS1299_CMD_STANDBY 0x04
/** ADS1299 RESET command opcode. */
#define ADS1299_CMD_RESET 0x06
/** ADS1299 START command opcode. */
#define ADS1299_CMD_START 0x08
/** ADS1299 STOP command opcode. */
#define ADS1299_CMD_STOP 0x0A
/** ADS1299 RDATAC command opcode. */
#define ADS1299_CMD_RDATAC 0x10
/** ADS1299 SDATAC command opcode. */
#define ADS1299_CMD_SDATAC 0x11
/** ADS1299 RDATA command opcode. */
#define ADS1299_CMD_RDATA 0x12
/** ADS1299 RREG command base opcode; OR with the register address. */
#define ADS1299_CMD_RREG 0x20
/** ADS1299 WREG command base opcode; OR with the register address. */
#define ADS1299_CMD_WREG 0x40

/** ADS1299 ID register address. */
#define ADS1299_REG_ID 0x00
/** ADS1299 CONFIG1 register address. */
#define ADS1299_REG_CONFIG1 0x01
/** ADS1299 CONFIG2 register address. */
#define ADS1299_REG_CONFIG2 0x02
/** ADS1299 CONFIG3 register address. */
#define ADS1299_REG_CONFIG3 0x03
/** ADS1299 lead-off control register address. */
#define ADS1299_REG_LOFF 0x04
/** ADS1299 channel 1 settings register address. */
#define ADS1299_REG_CH1SET 0x05
/** ADS1299 channel 2 settings register address. */
#define ADS1299_REG_CH2SET 0x06
/** ADS1299 channel 3 settings register address. */
#define ADS1299_REG_CH3SET 0x07
/** ADS1299 channel 4 settings register address. */
#define ADS1299_REG_CH4SET 0x08
/** ADS1299 channel 5 settings register address. */
#define ADS1299_REG_CH5SET 0x09
/** ADS1299 channel 6 settings register address. */
#define ADS1299_REG_CH6SET 0x0A
/** ADS1299 channel 7 settings register address. */
#define ADS1299_REG_CH7SET 0x0B
/** ADS1299 channel 8 settings register address. */
#define ADS1299_REG_CH8SET 0x0C
/** ADS1299 positive BIAS sense register address. */
#define ADS1299_REG_BIAS_SENSP 0x0D
/** ADS1299 negative BIAS sense register address. */
#define ADS1299_REG_BIAS_SENSN 0x0E

/**
 * @brief ADS1299 output data rates.
 *
 * Values match CONFIG1 register bits [2:0].
 */
typedef enum {
    ADS1299_DR_16kSPS = 0x00,                    /**< 16000 samples per second. */
    ADS1299_DR_8kSPS = 0x01,                     /**< 8000 samples per second. */
    ADS1299_DR_4kSPS = 0x02,                     /**< 4000 samples per second. */
    ADS1299_DR_2kSPS = 0x03,                     /**< 2000 samples per second. */
    ADS1299_DR_1kSPS = 0x04,                     /**< 1000 samples per second. */
    ADS1299_DR_500SPS = 0x05,                    /**< 500 samples per second. */
    ADS1299_DR_250SPS = 0x06                     /**< 250 samples per second. */
} ads1299_sample_rate_t;

/**
 * @brief ADS1299 programmable gain amplifier settings.
 *
 * Values match CHnSET register bits [6:4].
 */
typedef enum {
    ADS1299_PGA_GAIN_1 = 0x00,                   /**< Gain of 1. */
    ADS1299_PGA_GAIN_2 = 0x10,                   /**< Gain of 2. */
    ADS1299_PGA_GAIN_4 = 0x20,                   /**< Gain of 4. */
    ADS1299_PGA_GAIN_6 = 0x30,                   /**< Gain of 6. */
    ADS1299_PGA_GAIN_8 = 0x40,                   /**< Gain of 8. */
    ADS1299_PGA_GAIN_12 = 0x50,                  /**< Gain of 12. */
    ADS1299_PGA_GAIN_24 = 0x60                   /**< Gain of 24. */
} ads1299_pga_gain_t;

/**
 * @brief ADS1299 channel input mux settings.
 *
 * Values match CHnSET register bits [2:0].
 */
typedef enum {
    ADS1299_INPUT_NORMAL = 0x00,                 /**< Normal electrode input. */
    ADS1299_INPUT_SHORTED = 0x01,                /**< Shorted input for noise measurements. */
    ADS1299_INPUT_BIAS_MEAS = 0x02,              /**< BIAS drive measurement input. */
    ADS1299_INPUT_MVDD = 0x03,                   /**< Supply voltage measurement input. */
    ADS1299_INPUT_TEMP = 0x04,                   /**< Internal temperature sensor input. */
    ADS1299_INPUT_TEST_SIGNAL = 0x05             /**< Internal test signal input. */
} ads1299_input_mux_t;

#ifdef __cplusplus
}
#endif

#endif /* ADS1299_DEFS_H */
