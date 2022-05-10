// Defines for the PiicoDev BME280 atmospheric sensor.
#include "hw_i2c_bus.h"

// Constants and register addresses
#define BME280_DEFAULT_I2C_ADDR _u(0x77)
#define BME280_BME_CHIPID _u(0x60)
#define BME280_BME_RESET_FLAG _u(0xB6)

#define BME280_REG_CHIPID _u(0xD0)
#define BME280_REG_RESET _u(0xE0)
#define BME280_REG_STATUS _u(0xF3)
#define BME280_REG_CTRL_HUM _u(0xF2)
#define BME280_REG_CTRL_MEAS _u(0xF4)
#define BME280_REG_CONFIG _u(0xF5)
#define BME280_REG_PRESSURE_MSB _u(0xF7)
#define BME280_REG_DIG_T1_LSB _u(0x88)
#define BME280_REG_DIG_H1 _u(0xA1)
#define BME280_REG_DIG_H2 _u(0xE1)

#define BME280_NUM_TP_CALIB_BYTES 24
#define BME280_NUM_H_CALIB_BYTES 7

#define BME280_OVERSAMPLING_OFF _u(0x00)
#define BME280_OVERSAMPLING_x1 _u(0x01)
#define BME280_OVERSAMPLING_x2 _u(0x02)
#define BME280_OVERSAMPLING_x4 _u(0x03)
#define BME280_OVERSAMPLING_x8 _u(0x04)
#define BME280_OVERSAMPLING_x16 _u(0x05)

// Codes for measurement standby time. Each one is BME280_STANDBY_millisecs_tenths
#define BME280_STANDBY_0_5 0
#define BME280_STANDBY_62_5 1
#define BME280_STANDBY_125 2
#define BME280_STANDBY_250 3
#define BME280_STANDBY_500 4
#define BME280_STANDBY_1000 5
#define BME280_STANDBY_10 6
#define BME280_STANDBY_20 7

// Codes for number of samples in IIR filter.
#define BME280_IIR_FILTER_OFF 0
#define BME280_IIR_FILTER_2 1
#define BME280_IIR_FILTER_4 2
#define BME280_IIR_FILTER_8 3
#define BME280_IIR_FILTER_16 4

#define BME280_TEMP 0
#define BME280_PRES 1
#define BME280_HUM 2

// Device modes
#define BME280_DEV_SLEEP 0
#define BME280_DEV_FORCED 1
#define BME280_DEV_NORMAL 3

#define BME280_BUSY_MASK _u(0x08)

typedef struct {
    // temperature params
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;

    // pressure params
    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;

    // humidity params
    uint8_t dig_h1;
    int16_t dig_h2;
    uint8_t dig_h3;
    int16_t dig_h4;
    int16_t dig_h5;
    int8_t dig_h6;

}  bme280_calib_param_t;

typedef struct bme280_inst {
    i2c_inst_t* i2c_bus;
    uint8_t addr;
    bme280_calib_param_t* calib;
} bme280_inst_t;


bme280_inst_t* hw_BME280_init(i2c_inst_t* i2c_bus);
void hw_BME280_get_calib_params(bme280_inst_t* bme280);
void hw_BME280_set_data_acq_options(bme280_inst_t* bme280);
void hw_BME280_read_raw(bme280_inst_t* bme280, int32_t* temp, int32_t* pressure, int32_t* humidity);
bool hw_BME280_is_busy(bme280_inst_t* bme280);
int32_t hw_BME280_convert_temp(bme280_inst_t* bme280, int32_t adc_T);
int32_t hw_BME280_convert_pressure(bme280_inst_t* bme280, int32_t adc_P, int32_t adc_T);
int32_t hw_BME280_convert_humidity(bme280_inst_t* bme280, int32_t adc_H, int32_t adc_T);
void hw_BME280_read_values(bme280_inst_t* bme280, 
                            float* temperature, float* pressure, float* humidity);