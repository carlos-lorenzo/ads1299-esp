### Pin Mapping (ESP32-S3 to ADS1299 / Peripherals)

| Signal Name   | ESP32-S3 GPIO | Description / Function                         |
|:--------------|:-------------:|:-----------------------------------------------|
| **SDA_BAT**   |     GPIO4     | I2C Data (Battery Management / Sensor)         |
| **SCL_BAT**   |     GPIO5     | I2C Clock (Battery Management / Sensor)        |
| **EN_AP**     |     GPIO6     | Enable / Power Control                         |
| **SCL_IMU**   |     GPIO7     | I2C Clock (Inertial Measurement Unit / IMU)    |
| **DRDY_ADC**  |     GPIO8     | Data Ready (ADS1299)                           |
| **MISO_ADC**  |     GPIO9     | SPI Master In Slave Out (ADS1299)              |
| **SCLK_ADC**  |    GPIO10     | SPI Serial Clock (ADS1299)                     |
| **CS1_ADC**   |    GPIO11     | Chip Select 1 (ADS1299)                        |
| **START_ADC** |    GPIO12     | Start Conversion (ADS1299)                     |
| **RESET_ADC** |    GPIO13     | Reset (ADS1299)                                |
| **MOSI_ADC**  |    GPIO14     | SPI Master Out Slave In (ADS1299)              |
| **SDA_IMU**   |    GPIO15     | I2C Data (Inertial Measurement Unit / IMU)     |
| **CS_IMU**    |    GPIO16     | Chip Select (IMU SPI Mode, if used)            |
| **INT2_IMU**  |    GPIO17     | Interrupt 2 (IMU)                              |
| **INT1_IMU**  |    GPIO18     | Interrupt 1 (IMU)                              |
| **CS2_ADC**   |    GPIO21     | Chip Select 2 (Second ADS1299 - Cascade Mode ) |