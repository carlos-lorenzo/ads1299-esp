#ifndef ADS1299_DEFS_H
#define ADS1299_DEFS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


/**
 * @brief ADS1299 timing constants
 * Assuming an internal clock of 2.048 MHz
 */
#define ADS1299_CLK_FREQ      2048000UL

// Convert clock cycles to microseconds, rounding up
#define ADS1299_TICKS_TO_US(ticks) (((ticks) * 1000000UL + (ADS1299_CLK_FREQ - 1)) / ADS1299_CLK_FREQ)


#define ADS1299_T_CLK_US ADS1299_TICKS_TO_US(1)
#define ADS1299_T_RESET ADS1299_TICKS_TO_US(18)
#define ADS1299_T_PULSE ADS1299_TICKS_TO_US(2)
#define ADS1299_T_SDATAC ADS1299_TICKS_TO_US(2)
#define ADS1299_T_RDATAC ADS1299_TICKS_TO_US(2)
#define ADS1299_T_WAKEUP ADS1299_TICKS_TO_US(4)


#define ADS1299_T_CSSC ADS1299_TICKS_TO_US(1)
#define ADS1299_T_SCLK ADS1299_TICKS_TO_US(1)

#define ADS1299_T_CSH ADS1299_TICKS_TO_US(2)
#define ADS1299_T_SCCS ADS1299_TICKS_TO_US(4)
#define ADS1299_T_SDECODE ADS1299_TICKS_TO_US(4)


#define ADS1299_FRAME_SIZE 27  // 3 status + 8 channels * 3 bytes

/**
 * @brief ADS1299 SPI Command Opcodes (Datasheet Table 12)
 * These are fixed single-byte or multi-byte protocol bytes.
 */
#define ADS1299_CMD_WAKEUP   0x02
#define ADS1299_CMD_STANDBY  0x04
#define ADS1299_CMD_RESET    0x06
#define ADS1299_CMD_START    0x08
#define ADS1299_CMD_STOP     0x0A
#define ADS1299_CMD_RDATAC   0x10
#define ADS1299_CMD_SDATAC   0x11
#define ADS1299_CMD_RDATA    0x12
#define ADS1299_CMD_RREG     0x20  // Combine with register address: (0x20 | reg)
#define ADS1299_CMD_WREG     0x40  // Combine with register address: (0x40 | reg)

/**
 * @brief ADS1299 Register Addresses (Datasheet Table 14)
 */
#define ADS1299_REG_ID       0x00
#define ADS1299_REG_CONFIG1  0x01
#define ADS1299_REG_CONFIG2  0x02
#define ADS1299_REG_CONFIG3  0x03
#define ADS1299_REG_LOFF     0x04
#define ADS1299_REG_CH1SET   0x05
#define ADS1299_REG_CH2SET   0x06
#define ADS1299_REG_CH3SET   0x07
#define ADS1299_REG_CH4SET   0x08
#define ADS1299_REG_CH5SET   0x09
#define ADS1299_REG_CH6SET   0x0A
#define ADS1299_REG_CH7SET   0x0B
#define ADS1299_REG_CH8SET   0x0C
#define ADS1299_REG_BIAS_SENSP 0x0D
#define ADS1299_REG_BIAS_SENSN 0x0E

/**
 * @brief Configuration Enums for Type Safety
 * Matching the exact bit shifts required by the registers.
 */

/**
 * @brief Data Output Rates (CONFIG1 Register Bits [2:0])
 * For EEG, 250 or 500 SPS are standard to reduce ambient noise.
 */
typedef enum {
    ADS1299_DR_16kSPS  = 0x00,
    ADS1299_DR_8kSPS   = 0x01,
    ADS1299_DR_4kSPS   = 0x02,
    ADS1299_DR_2kSPS   = 0x03,
    ADS1299_DR_1kSPS   = 0x04,
    ADS1299_DR_500SPS  = 0x05,
    ADS1299_DR_250SPS  = 0x06
} ads1299_sample_rate_t;

/**
 * @brief Programmable Gain Amplifier (PGA) Settings (CHxSET Register Bits [6:4])
 * EEG signals are tiny (~10-100uV), so you'll usually want a high gain like 12 or 24.
 */
typedef enum {
    ADS1299_PGA_GAIN_1  = 0x00,
    ADS1299_PGA_GAIN_2  = 0x10,
    ADS1299_PGA_GAIN_4  = 0x20,
    ADS1299_PGA_GAIN_6  = 0x30,
    ADS1299_PGA_GAIN_8  = 0x40,
    ADS1299_PGA_GAIN_12 = 0x50,
    ADS1299_PGA_GAIN_24 = 0x60
} ads1299_pga_gain_t;

/**
 * @brief Channel Input Mux Options (CHxSET Register Bits [2:0])
 */
typedef enum {
    ADS1299_INPUT_NORMAL      = 0x00, // Normal electrode input
    ADS1299_INPUT_SHORTED     = 0x01, // Used for noise measurements
    ADS1299_INPUT_BIAS_MEAS   = 0x02, // Used for Bias drive measurement
    ADS1299_INPUT_MVDD        = 0x03, // Supply voltage measurement
    ADS1299_INPUT_TEMP        = 0x04, // Internal temperature sensor
    ADS1299_INPUT_TEST_SIGNAL = 0x05  // Internal 1Hz square wave calibration signal
} ads1299_input_mux_t;

#ifdef __cplusplus
}
#endif

#endif // ADS1299_DEFS_H