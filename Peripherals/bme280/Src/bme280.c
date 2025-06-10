#include "bme280.h"


typedef struct __attribute__((packed))
{
  uint8_t press_msb; // 0xF7
  uint8_t press_lsb; // 0xF8
  uint8_t press_xlsb; // 0xF9
  uint8_t temp_msb; // 0xFA
  uint8_t temp_lsb; // 0xFB
  uint8_t temp_xlsb; // 0xFC
  uint8_t hum_msb; // 0xFD
  uint8_t hum_lsb; // 0xFE
} bme280_raw_data_t;

static bme280_result_t bme280_self_test (const bme280_t *const dev);
static bme280_result_t bme280_read_calibration_data (bme280_t *dev);
static bme280_result_t bme280_set_mode (const bme280_t *const dev,
                                        bme280_mode_t mode);
static bme280_result_t bme280_read_raw_data (const bme280_t *const dev,
                                             bme280_raw_data_t *raw_data);
static uint32_t bme280_extract_pressure (const bme280_raw_data_t *raw_data);
static uint32_t bme280_extract_temperature (const bme280_raw_data_t *raw_data);
static uint16_t bme280_extract_humidity (const bme280_raw_data_t *raw_data);
static int32_t bme280_compensate_temperature (const bme280_t *dev,
                                              int32_t adc_T, int32_t *t_fine);
static uint32_t bme280_compensate_pressure (const bme280_t *dev, int32_t adc_P,
                                            int32_t t_fine);
static uint32_t bme280_compensate_humidity (const bme280_t *dev, int32_t adc_H,
                                            int32_t t_fine);


static bme280_result_t
bme280_self_test (const bme280_t *const dev)
{
  uint8_t chip_id = 0;

  const HAL_StatusTypeDef status = HAL_I2C_Mem_Read (
      dev->hi2c,
      dev->i2c_addr << 1,
      BME280_REG_CHIP_ID,
      I2C_MEMADD_SIZE_8BIT,
      &chip_id,
      sizeof(chip_id),
      HAL_MAX_DELAY);

  if (status != HAL_OK)
    {
      return BME280_E_COMM_FAIL;
    }

  if (chip_id != BME280_CHIP_ID)
    {
      return BME280_E_INVALID_CHIP_ID;
    }

  return BME280_OK;
}

static bme280_result_t
bme280_read_calibration_data (bme280_t *dev)
{
  uint8_t calib_data[26];
  uint8_t calib_data_h[7];

  //  temp pres calib 0x88-0xA1
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read (
      dev->hi2c,
      dev->i2c_addr << 1,
      0x88,
      I2C_MEMADD_SIZE_8BIT,
      calib_data,
      26,
      HAL_MAX_DELAY);

  if (status != HAL_OK)
    {
      return BME280_E_COMM_FAIL;
    }

  // hum calib 0xE1-0xE7
  status = HAL_I2C_Mem_Read (
      dev->hi2c,
      dev->i2c_addr << 1,
      0xE1,
      I2C_MEMADD_SIZE_8BIT,
      calib_data_h,
      7,
      HAL_MAX_DELAY);

  if (status != HAL_OK)
    {
      return BME280_E_COMM_FAIL;
    }

  dev->calib.dig_T1 = (calib_data[1] << 8) | calib_data[0];
  dev->calib.dig_T2 = (calib_data[3] << 8) | calib_data[2];
  dev->calib.dig_T3 = (calib_data[5] << 8) | calib_data[4];

  dev->calib.dig_P1 = (calib_data[7] << 8) | calib_data[6];
  dev->calib.dig_P2 = (calib_data[9] << 8) | calib_data[8];
  dev->calib.dig_P3 = (calib_data[11] << 8) | calib_data[10];
  dev->calib.dig_P4 = (calib_data[13] << 8) | calib_data[12];
  dev->calib.dig_P5 = (calib_data[15] << 8) | calib_data[14];
  dev->calib.dig_P6 = (calib_data[17] << 8) | calib_data[16];
  dev->calib.dig_P7 = (calib_data[19] << 8) | calib_data[18];
  dev->calib.dig_P8 = (calib_data[21] << 8) | calib_data[20];
  dev->calib.dig_P9 = (calib_data[23] << 8) | calib_data[22];

  dev->calib.dig_H1 = calib_data[25];
  dev->calib.dig_H2 = (calib_data_h[1] << 8) | calib_data_h[0];
  dev->calib.dig_H3 = calib_data_h[2];
  dev->calib.dig_H4 = (calib_data_h[3] << 4) | (calib_data_h[4] & 0x0F);
  dev->calib.dig_H5 = (calib_data_h[5] << 4) | (calib_data_h[4] >> 4);
  dev->calib.dig_H6 = calib_data_h[6];

  return BME280_OK;
}

static bme280_result_t
bme280_set_mode (const bme280_t *const dev, bme280_mode_t mode)
{
  uint8_t ctrl_meas_reg_value = 0;
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read (
      dev->hi2c,
      dev->i2c_addr << 1,
      BME280_REG_CTRL_MEAS,
      I2C_MEMADD_SIZE_8BIT,
      &ctrl_meas_reg_value,
      sizeof(ctrl_meas_reg_value),
      HAL_MAX_DELAY);

  if (status != HAL_OK)
    {
      return BME280_E_COMM_FAIL;
    }

  ctrl_meas_reg_value = (ctrl_meas_reg_value & 0xFC) | (mode & 0x03);

  status = HAL_I2C_Mem_Write (
      dev->hi2c,
      dev->i2c_addr << 1,
      BME280_REG_CTRL_MEAS,
      I2C_MEMADD_SIZE_8BIT,
      &ctrl_meas_reg_value,
      sizeof(ctrl_meas_reg_value),
      HAL_MAX_DELAY);

  if (status != HAL_OK)
    {
      return BME280_E_COMM_FAIL;
    }

  return BME280_OK;
}

static bme280_result_t
bme280_read_raw_data (const bme280_t *const dev, bme280_raw_data_t *raw_data)
{
  const HAL_StatusTypeDef status = HAL_I2C_Mem_Read (
      dev->hi2c,
      dev->i2c_addr << 1,
      BURST_READ_START_ADDR,
      I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)raw_data,
      sizeof (bme280_raw_data_t),
      HAL_MAX_DELAY);

  if (status != HAL_OK)
    {
      return BME280_E_COMM_FAIL;
    }

  return BME280_OK;
}

static uint32_t
bme280_extract_pressure (const bme280_raw_data_t *raw_data)
{
  uint32_t pressure = ((uint32_t)raw_data->press_msb << 12) |
                      ((uint32_t)raw_data->press_lsb << 4) |
                      ((uint32_t)raw_data->press_xlsb >> 4);
  return pressure;
}

static uint32_t
bme280_extract_temperature (const bme280_raw_data_t *raw_data)
{
  uint32_t temperature = ((uint32_t)raw_data->temp_msb << 12) |
                         ((uint32_t)raw_data->temp_lsb << 4) |
                         ((uint32_t)raw_data->temp_xlsb >> 4);
  return temperature;
}

static uint16_t
bme280_extract_humidity (const bme280_raw_data_t *raw_data)
{
  uint16_t humidity = ((uint16_t)raw_data->hum_msb << 8) |
                      (uint16_t)raw_data->hum_lsb;
  return humidity;
}

static int32_t
bme280_compensate_temperature (const bme280_t *dev, int32_t adc_T,
                               int32_t *t_fine)
{
  int32_t var1 = ((((adc_T >> 3) - ((int32_t)dev->calib.dig_T1 << 1))) *
                  ((int32_t)dev->calib.dig_T2)) >> 11;

  int32_t var2 = (((((adc_T >> 4) - ((int32_t)dev->calib.dig_T1)) *
                    ((adc_T >> 4) - ((int32_t)dev->calib.dig_T1))) >> 12) *
                  ((int32_t)dev->calib.dig_T3)) >> 14;

  *t_fine = var1 + var2;
  int32_t T = (*t_fine * 5 + 128) >> 8;

  return T; // 0.01C
}

static uint32_t
bme280_compensate_pressure (const bme280_t *dev, int32_t adc_P, int32_t t_fine)
{
  int64_t var1 = ((int64_t)t_fine) - 128000;
  int64_t var2 = var1 * var1 * (int64_t)dev->calib.dig_P6;
  var2 = var2 + ((var1 * (int64_t)dev->calib.dig_P5) << 17);
  var2 = var2 + (((int64_t)dev->calib.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)dev->calib.dig_P3) >> 8) +
         ((var1 * (int64_t)dev->calib.dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dev->calib.dig_P1) >> 33;

  if (var1 == 0)
    {
      return 0;
    }

  int64_t p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)dev->calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)dev->calib.dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)dev->calib.dig_P7) << 4);

  return (uint32_t)p; //pa
}

static uint32_t
bme280_compensate_humidity (const bme280_t *dev, int32_t adc_H, int32_t t_fine)
{
  int32_t v_x1_u32r = (t_fine - ((int32_t)76800));
  v_x1_u32r = (((((adc_H << 14) - (((int32_t)dev->calib.dig_H4) << 20) -
                  (((int32_t)dev->calib.dig_H5) * v_x1_u32r)) +
                 ((int32_t)16384)) >> 15) *
               (((((((v_x1_u32r * ((int32_t)dev->calib.dig_H6)) >> 10) *
                    (((v_x1_u32r * ((int32_t)dev->calib.dig_H3)) >> 11) +
                     ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
                 ((int32_t)dev->calib.dig_H2) + 8192) >> 14));

  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                             ((int32_t)dev->calib.dig_H1)) >> 4));

  v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
  v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

  return (uint32_t)(v_x1_u32r >> 12);
}

bme280_result_t
bme280_init (bme280_t *dev,
             I2C_HandleTypeDef *hi2c,
             uint8_t i2c_addr,
             bme280_mode_t mode)
{
  if (dev == NULL || hi2c == NULL)
    {
      return BME280_E_NULL_PTR;
    }

  dev->hi2c = hi2c;
  dev->i2c_addr = i2c_addr;
  dev->mode = mode;

  bme280_result_t result = bme280_self_test (dev);
  if (result != BME280_OK)
    {
      return result;
    }

  result = bme280_read_calibration_data (dev);
  if (result != BME280_OK)
    {
      return result;
    }

  result = bme280_set_mode (dev, mode);
  if (result != BME280_OK)
    {
      return result;
    }

  return BME280_OK;
}

bme280_result_t
bme280_read_data (const bme280_t *dev, bme280_data_t *data)
{
  if (dev == NULL || data == NULL)
    {
      return BME280_E_NULL_PTR;
    }

  bme280_raw_data_t raw_data;
  bme280_result_t result = bme280_read_raw_data (dev, &raw_data);

  if (result != BME280_OK)
    {
      return result;
    }

  uint32_t temp_raw = bme280_extract_temperature (&raw_data);
  uint32_t press_raw = bme280_extract_pressure (&raw_data);
  uint16_t hum_raw = bme280_extract_humidity (&raw_data);

  int32_t t_fine;
  int32_t temp_comp = bme280_compensate_temperature (dev, temp_raw, &t_fine);
  data->temperature = temp_comp / 100.0f;

  uint32_t press_comp = bme280_compensate_pressure (dev, press_raw, t_fine);
  data->pressure = press_comp / 100.0f;

  uint32_t hum_comp = bme280_compensate_humidity (dev, hum_raw, t_fine);
  data->humidity = hum_comp / 1024.0f;

  return BME280_OK;
}

bme280_result_t
bme280_read_temperature (const bme280_t *dev, float *temperature)
{
  if (dev == NULL || temperature == NULL)
    {
      return BME280_E_NULL_PTR;
    }

  bme280_raw_data_t raw_data;
  bme280_result_t result = bme280_read_raw_data (dev, &raw_data);

  if (result != BME280_OK)
    {
      return result;
    }

  uint32_t temp_raw = bme280_extract_temperature (&raw_data);

  int32_t t_fine;
  int32_t temp_comp = bme280_compensate_temperature (dev, temp_raw, &t_fine);
  *temperature = temp_comp / 100.0f;

  return BME280_OK;
}

bme280_result_t
bme280_read_humidity (const bme280_t *dev, float *humidity)
{
  if (dev == NULL || humidity == NULL)
    {
      return BME280_E_NULL_PTR;
    }

  bme280_raw_data_t raw_data;
  bme280_result_t result = bme280_read_raw_data (dev, &raw_data);

  if (result != BME280_OK)
    {
      return result;
    }

  uint32_t temp_raw = bme280_extract_temperature (&raw_data);
  uint16_t hum_raw = bme280_extract_humidity (&raw_data);

  int32_t t_fine;
  bme280_compensate_temperature (dev, temp_raw, &t_fine);

  uint32_t hum_comp = bme280_compensate_humidity (dev, hum_raw, t_fine);
  *humidity = hum_comp / 1024.0f;

  return BME280_OK;
}