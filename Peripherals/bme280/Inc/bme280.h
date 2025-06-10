#ifndef MUSHROOM_BME280_H
#define MUSHROOM_BME280_H

#include "stm32f0xx_hal.h"

#define BME280_I2C_ADDR_PRIMARY     (0x76)
#define BME280_I2C_ADDR_SECONDARY   (0x77)

#define BME280_CHIP_ID              (0x60)

#define BME280_REG_CHIP_ID          (0xD0)
#define BME280_REG_RESET            (0xE0)

#define BME280_REG_CTRL_HUM         (0xF2)
#define BME280_REG_CTRL_MEAS        (0xF4)

#define BME280_REG_CONFIG           (0xF5)

#define BME280_REG_TEMPERATURE_MSB  (0xFA)
#define BME280_REG_TEMPERATURE_LSB  (0xFB)
#define BME280_REG_TEMPERATURE_XLSB (0xFC)
#define BME280_REG_HUMIDITY_MSB     (0xFD)
#define BME280_REG_HUMIDITY_LSB     (0xFE)

#define BURST_READ_START_ADDR 0xF7

typedef enum
{
  BME280_MODE_SLEEP = 0x00,
  BME280_MODE_FORCED = 0x01,
  BME280_MODE_NORMAL = 0x11,
} bme280_mode_t;

typedef enum
{
  BME280_OK = 0,
  BME280_E_COMM_FAIL,
  BME280_E_NULL_PTR,
  BME280_E_DEV_NOT_FOUND,
  BME280_E_INVALID_PARAM,
  BME280_E_INVALID_STATE,
  BME280_E_INVALID_CALIB_DATA,
  BME280_E_INVALID_CHIP_ID,
  BME280_E_INVALID_DATA,
} bme280_result_t;

typedef struct
{
  float temperature;
  float humidity;
  float pressure;
} bme280_data_t;

typedef struct
{
  // temperature calibration
  uint16_t dig_T1;
  int16_t dig_T2;
  int16_t dig_T3;

  // pressure calibration
  uint16_t dig_P1;
  int16_t dig_P2;
  int16_t dig_P3;
  int16_t dig_P4;
  int16_t dig_P5;
  int16_t dig_P6;
  int16_t dig_P7;
  int16_t dig_P8;
  int16_t dig_P9;

  // humidity calibration
  uint8_t dig_H1;
  int16_t dig_H2;
  uint8_t dig_H3;
  int16_t dig_H4;
  int16_t dig_H5;
  int8_t dig_H6;
} bme280_calib_data_t;


typedef struct
{
  I2C_HandleTypeDef *hi2c;
  uint8_t i2c_addr;
  bme280_mode_t mode;
  bme280_calib_data_t calib;
} bme280_t;

bme280_result_t bme280_init (bme280_t *dev,
                             I2C_HandleTypeDef *hi2c,
                             uint8_t i2c_addr,
                             bme280_mode_t mode);

bme280_result_t bme280_read_data (
    bme280_t *dev, bme280_data_t *data);

bme280_result_t bme280_read_temperature (
    bme280_t *dev, float *temperature);

bme280_result_t bme280_read_humidity (
    bme280_t *dev, float *humidity);


#endif /* MUSHROOM_BME280_H */