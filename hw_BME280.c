// hw_BME280.c - Functions to access and control the PiicoDev BME280 Atmospheric sensor.

#include <stdio.h>
#include "hw_BME280.h"

static bme280_inst_t bme280_inst;
static bme280_calib_param_t bme280_calib;

bme280_inst_t* hw_BME280_init(i2c_inst_t* i2c_bus) {
    bme280_inst_t* bme280 = {&bme280_inst};
    
    bme280->i2c_bus = i2c_bus;
    bme280->addr = BME280_DEFAULT_I2C_ADDR;
    bme280->calib = &bme280_calib;

    // Check to see we have a BME280 at this address.
    uint8_t reg = BME280_REG_CHIPID;
    uint8_t data;
    if (i2c_readfrom_mem(bme280->i2c_bus, bme280->addr, &reg, sizeof reg, &data, sizeof data))
    {
        if (data != BME280_BME_CHIPID) 
        {
            printf("?? Chip ID is 0x%2X - expected 0x%2X\n", data, BME280_BME_CHIPID);
        }
    }

    // Reset the device.
    i2c_write8(bme280->i2c_bus, bme280->addr, BME280_REG_RESET, BME280_BME_RESET_FLAG );
    hw_BME280_set_data_acq_options(bme280);
    hw_BME280_get_calib_params(bme280);

    return bme280;
}

void hw_BME280_set_data_acq_options(bme280_inst_t* bme280) {
    // Set the data acquisition options.
    uint8_t sampling_mode = BME280_DEV_FORCED;
    uint8_t standby_time = BME280_STANDBY_125;

    // Configure the oversampling modes for adc_H, temperature and pressure.
    // osrs_h x1
    const uint8_t reg_ctrl_hum_val = BME280_OVERSAMPLING_x1;
    i2c_write8(bme280->i2c_bus, bme280->addr, BME280_REG_CTRL_HUM, reg_ctrl_hum_val);

    // osrs_t x1, osrs_p x1, forced mode operation
    const uint8_t reg_ctrl_meas_val = (BME280_OVERSAMPLING_x1 << 5) | (BME280_OVERSAMPLING_x1 << 2) | (sampling_mode);
    i2c_write8(bme280->i2c_bus, bme280->addr, BME280_REG_CTRL_MEAS, reg_ctrl_meas_val);

    // Set standby time and IIR filter
    uint8_t reg_config_val;

    if (sampling_mode == BME280_DEV_NORMAL)
    {
        reg_config_val = (standby_time << 5);
    } else {
        reg_config_val = 0;
    }
    
    reg_config_val = (reg_config_val | (BME280_IIR_FILTER_OFF << 2));
    i2c_write8(bme280->i2c_bus, bme280->addr, BME280_REG_CONFIG, reg_config_val);
}

bool hw_BME280_is_busy(bme280_inst_t* bme280) {
    // returns true if the device is busy making a measurement.
    uint8_t reg = BME280_REG_STATUS;
    uint8_t status;
    i2c_readfrom_mem(bme280->i2c_bus, bme280->addr, &reg, 1, &status, 1);
    bool busy = (status & BME280_BUSY_MASK) >> 3;
    return busy;
}

void hw_BME280_get_calib_params(bme280_inst_t* bme280) {
    // raw temp and pressure values need to be calibrated according to
    // parameters generated during the manufacturing of the sensor
    // there are 3 temperature params, and 9 pressure params, each with a LSB
    // and MSB register, so we read from 24 registers

    uint8_t reg = BME280_REG_DIG_T1_LSB;
    uint8_t buf[BME280_NUM_TP_CALIB_BYTES] = { 0 };
    
    i2c_readfrom_mem(bme280->i2c_bus, bme280->addr, &reg, 1, buf, BME280_NUM_TP_CALIB_BYTES);

    // store these in a struct for later use
    bme280_calib_param_t* calib = bme280->calib;

    calib->dig_t1 = (uint16_t)(buf[1] << 8) | buf[0];
    calib->dig_t2 = (int16_t)(buf[3] << 8) | buf[2];
    calib->dig_t3 = (int16_t)(buf[5] << 8) | buf[4];

    calib->dig_p1 = (uint16_t)(buf[7] << 8) | buf[6];
    calib->dig_p2 = (int16_t)(buf[9] << 8) | buf[8];
    calib->dig_p3 = (int16_t)(buf[11] << 8) | buf[10];
    calib->dig_p4 = (int16_t)(buf[13] << 8) | buf[12];
    calib->dig_p5 = (int16_t)(buf[15] << 8) | buf[14];
    calib->dig_p6 = (int16_t)(buf[17] << 8) | buf[16];
    calib->dig_p7 = (int16_t)(buf[19] << 8) | buf[18];
    calib->dig_p8 = (int16_t)(buf[21] << 8) | buf[20];
    calib->dig_p9 = (int16_t)(buf[23] << 8) | buf[22];

    // now get the adc_H calibration parameters.
    reg = BME280_REG_DIG_H1;
    uint8_t buf1[BME280_NUM_H_CALIB_BYTES] = { 0 };
    
    i2c_readfrom_mem(bme280->i2c_bus, bme280->addr, &reg, 1, buf1, 1);
    calib->dig_h1 = buf1[0];

    reg = BME280_REG_DIG_H2;
    i2c_readfrom_mem(bme280->i2c_bus, bme280->addr, &reg, 1, buf1, BME280_NUM_H_CALIB_BYTES);
    calib->dig_h2 = (int16_t)(buf1[1] << 8) | buf1[0];
    calib->dig_h3 = buf1[2];
    calib->dig_h4 = (int16_t)(buf1[3] << 4) | (buf1[4] & 0xF);
    calib->dig_h5 = (int16_t)(buf1[5] << 4) | (buf1[4] >> 4);
    calib->dig_h6 = (int8_t)buf1[6];

}

void hw_BME280_read_raw(bme280_inst_t* bme280, int32_t* adc_T, int32_t* adc_P, int32_t* adc_H) {
    // BME280 data registers are auto-incrementing and we have 3 temperature and
    // pressure registers each and then two humidity registers, so we start at 0xF7 and read 8 bytes to 0xFE
    // note: normal mode does not require further ctrl_meas and config register writes

    hw_BME280_set_data_acq_options(bme280);
    int spins = 0;
    while (hw_BME280_is_busy(bme280)) {
        spins++;
        sleep_ms(10);
    }
    //printf(" Spins = %d\n", spins);

    uint8_t reg = BME280_REG_PRESSURE_MSB;
    uint8_t buf[8];
    i2c_readfrom_mem(bme280->i2c_bus, bme280->addr, &reg, 1, buf, 8);

    // convert the bytes to two 20 bit values and one 16 bit value.
    // store each in a 32 bit signed integer for conversion
    *adc_P = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
    *adc_T = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);
    *adc_H = (buf[6] << 8) | (buf[7]);
}

// intermediate function that calculates the fine resolution temperature
// used for both pressure and temperature conversions
int32_t hw_BME280_convert(bme280_inst_t* bme280, int32_t adc_T) {
    // use the 32-bit fixed point compensation implementation given in the
    // datasheet
    
    int32_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int32_t)bme280->calib->dig_t1 << 1))) * ((int32_t)bme280->calib->dig_t2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)bme280->calib->dig_t1)) * ((adc_T >> 4) 
            - ((int32_t)bme280->calib->dig_t1))) >> 12) 
            * ((int32_t)bme280->calib->dig_t3)) >> 14;
    return var1 + var2;
}

int32_t hw_BME280_convert_temp(bme280_inst_t* bme280, int32_t adc_T) {
    // uses the BMP280 calibration parameters to compensate the temperature value read from its registers
    int32_t t_fine = hw_BME280_convert(bme280, adc_T);
    return (t_fine * 5 + 128) >> 8;
}

int32_t hw_BME280_convert_pressure(bme280_inst_t* bme280, int32_t adc_P, int32_t adc_T) {
    // uses the BMP280 calibration parameters to compensate the pressure value read from its registers

    int32_t t_fine = hw_BME280_convert(bme280, adc_T);

    int32_t var1, var2;
    uint32_t converted = 0;
    var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)bme280->calib->dig_p6);
    var2 += ((var1 * ((int32_t)bme280->calib->dig_p5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)bme280->calib->dig_p4) << 16);
    var1 = (((bme280->calib->dig_p3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) 
            + ((((int32_t)bme280->calib->dig_p2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)bme280->calib->dig_p1)) >> 15);
    if (var1 == 0) {
        return 0;  // avoid exception caused by division by zero
    }
    converted = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
    if (converted < 0x80000000) {
        converted = (converted << 1) / ((uint32_t)var1);
    } else {
        converted = (converted / (uint32_t)var1) * 2;
    }
    var1 = (((int32_t)bme280->calib->dig_p9) * ((int32_t)(((converted >> 3) * (converted >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(converted >> 2)) * ((int32_t)bme280->calib->dig_p8)) >> 13;
    converted = (uint32_t)((int32_t)converted + ((var1 + var2 + bme280->calib->dig_p7) >> 4));
    return converted;
}

int32_t hw_BME280_convert_humidity(bme280_inst_t* bme280, int32_t adc_H, int32_t adc_T) {
    // uses the BMP280 calibration parameters to compensate the pressure value read from its registers

    int32_t t_fine = hw_BME280_convert(bme280, adc_T);
    int32_t h = t_fine - 76800;
    int32_t var1, var2;

    var1 = (((adc_H<<14) - (bme280->calib->dig_h4<<20) - (bme280->calib->dig_h5 * h)) + 16384) >>15;
    var2 = (((((((h*bme280->calib->dig_h6)>>10) * (((h*bme280->calib->dig_h3)>>11) + 32768))>>10)
                + 2097152)*bme280->calib->dig_h2 + 8192)>>14);

    h = var1 * var2;
    h = h - (((((h>>15)*(h>>15))>>7)*bme280->calib->dig_h1)>>4);
    h = (h < 0 ? 0 : h);
    h = (h > 419430400 ? 419430400 : h);
    return (h>>12);
}

void hw_BME280_read_values(bme280_inst_t* bme280, 
                            float* temperature, float* pressure, float* humidity) {

        int32_t adc_T, adc_P, adc_H;

        hw_BME280_read_raw(bme280, &adc_T, &adc_P, &adc_H);
        int32_t temp_i32 = hw_BME280_convert_temp(bme280, adc_T);
        int32_t press_i32 = hw_BME280_convert_pressure(bme280, adc_P, adc_T);
        int32_t humid_i32 = hw_BME280_convert_humidity(bme280, adc_H, adc_T);

        *temperature = temp_i32 / 100.0F;
        *pressure = press_i32 / 100.0F;
        *humidity = humid_i32 / 1024.0F;
}
