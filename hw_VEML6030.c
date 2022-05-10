// hw_VEML6030.c - Functions to access and control the PiicoDev VEML6030 Atmospheric sensor.

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hw_VEML6030.h"

static veml6030_inst_t veml6030_inst;

veml6030_inst_t* hw_VEML6030_init(i2c_inst_t* i2c_bus, uint8_t address, bool auto_range) {

    veml6030_inst_t* veml6030 = {&veml6030_inst};

    veml6030->i2c_bus = i2c_bus;
    if ((address != VEML6030_I2C_ADDR_DEFAULT) & (address != VEML6030_I2C_ADDR_ALT)) {
        veml6030->addr  = VEML6030_I2C_ADDR_DEFAULT;
    } else {
        veml6030->addr = address;
    }

    // Power on the device and initialise the gain and integration time.
    i2c_write16(veml6030->i2c_bus, veml6030->addr, VEML6030_REG_ALS_CONF, VEML6030_ON);
    sleep_ms(4);

    veml6030->auto_range = auto_range;
    if (veml6030->auto_range) {
        hw_VEML6030_auto_tune(veml6030);
    } else {
        hw_VEML6030_set_gain(veml6030, 1.0/8);
        hw_VEML6030_set_integration_time(veml6030, 100);
    }
    
    return veml6030;
}

bool hw_VEML6030_turn_off(veml6030_inst_t* veml6030) {
  uint16_t config = i2c_read16(veml6030->i2c_bus, veml6030->addr, VEML6030_REG_ALS_CONF);
  config &= VEML6030_ONOFF_MASK;
  config |= VEML6030_OFF;
  return i2c_write16(veml6030->i2c_bus, veml6030->addr, VEML6030_REG_ALS_CONF, config);
}

bool hw_VEML6030_turn_on(veml6030_inst_t* veml6030) {
  uint16_t config = i2c_read16(veml6030->i2c_bus, veml6030->addr, VEML6030_REG_ALS_CONF);
  config &= VEML6030_ONOFF_MASK;
  return i2c_write16(veml6030->i2c_bus, veml6030->addr, VEML6030_REG_ALS_CONF, config);
}

bool hw_VEML6030_apply_config_parameter(veml6030_inst_t* veml6030, uint16_t mask, uint16_t param) {
    hw_VEML6030_turn_off(veml6030);
    uint16_t config = i2c_read16(veml6030->i2c_bus, veml6030->addr, VEML6030_REG_ALS_CONF);
    config &= ~mask;
    config |= param;
    config &= VEML6030_ONOFF_MASK;
    return i2c_write16(veml6030->i2c_bus, veml6030->addr, VEML6030_REG_ALS_CONF, config);
}

uint16_t hw_VEML6030_auto_tune(veml6030_inst_t* veml6030) {
    // Set the integration time and initial gain
    int gain_idx = 0;
    int integ_time_idx = 2;     // Start at 100 ms
    uint16_t integ_time_ms = veml6030_int_times[integ_time_idx].integ_time_ms;
    bool finished = false;
    uint16_t als_value;

    hw_VEML6030_set_integration_time(veml6030, integ_time_ms);

    // Tune gain first.
    while (!finished) {
        hw_VEML6030_set_gain(veml6030, veml6030_gains[gain_idx].gain);
        sleep_ms(integ_time_ms + 50);
        als_value = i2c_read16(veml6030->i2c_bus, veml6030->addr, VEML6030_REG_ALS);
        if (als_value >  VEML6030_THRESH_LOW) {
            finished = true;
        } else {
            gain_idx++;
            if (gain_idx >= VEML6030_NUM_GAIN_STEPS) {
                finished = true;
            }
        }
    }

    // If we have to, tune the integration time next
    if (als_value >  VEML6030_THRESH_HIGH) {         // If the sensor is already saturating start at the lowest integration time.
        integ_time_idx = -1;
        als_value = 0;
    }

    if (als_value <=  VEML6030_THRESH_LOW) {
        finished = false;
        integ_time_idx++;
        
        while (!finished) {
            integ_time_ms = veml6030_int_times[integ_time_idx].integ_time_ms;
            hw_VEML6030_set_integration_time(veml6030, veml6030_int_times[integ_time_idx].integ_time_ms);
            sleep_ms(integ_time_ms + 50);
            als_value = i2c_read16(veml6030->i2c_bus, veml6030->addr, VEML6030_REG_ALS);
            if (als_value >  VEML6030_THRESH_LOW) {
                finished = true;
            } else {
                integ_time_idx++;
                if (integ_time_idx >= VEML6030_NUM_INT_TIME_STEPS) {
                    finished = true;
                }
            }
        }
    }    

    veml6030->auto_range = true;    // Reset the flag to true as the set_gain and set_integration_time functions clear it.
    return als_value;
}

bool hw_VEML6030_set_integration_time(veml6030_inst_t* veml6030, uint16_t integ_time_ms)
{
    // Search for a matching integration time from the available values.
    veml6030->auto_range = false;
    veml6030->curr_integ_time_idx = (VEML6030_NUM_INT_TIME_STEPS)-1;

    for (int i=0; i < VEML6030_NUM_INT_TIME_STEPS; i++) {
        if (integ_time_ms <= veml6030_int_times[i].integ_time_ms) {
            veml6030->curr_integ_time_idx = i;
            break;
        }
    }

    veml6030->curr_integ_time_ms = veml6030_int_times[veml6030->curr_integ_time_idx].integ_time_ms;
    veml6030->lux_corr_factor = veml6030_gains[veml6030->curr_gain_idx].factor 
            * veml6030_int_times[veml6030->curr_integ_time_idx].factor * VEML6030_BASE_RESOLUTION;

    return hw_VEML6030_apply_config_parameter(veml6030, VEML6030_REG_CONF_INT_TIME_MASK,
                    veml6030_int_times[veml6030->curr_integ_time_idx].code);
}

uint16_t hw_VEML6030_get_integration_time(veml6030_inst_t* veml6030)
{
    uint16_t config = i2c_read16(veml6030->i2c_bus, veml6030->addr, VEML6030_REG_ALS_CONF);
    uint16_t code = config & VEML6030_REG_CONF_INT_TIME_MASK;

    veml6030->curr_integ_time_idx = (VEML6030_NUM_INT_TIME_STEPS)-1;

    for (int i=0; i < VEML6030_NUM_INT_TIME_STEPS; i++) {
        if (code == veml6030_int_times[i].code) {
            veml6030->curr_integ_time_idx = i;
            break;
        }
    }

    return veml6030_int_times[veml6030->curr_integ_time_idx].integ_time_ms;
}

bool hw_VEML6030_set_gain(veml6030_inst_t* veml6030, float gain)
{
    // Search for a matching gain from the available values.
    veml6030->auto_range = false;
    veml6030->curr_gain_idx = (VEML6030_NUM_GAIN_STEPS)-1;

    for (int i=0; i < VEML6030_NUM_GAIN_STEPS; i++) {
        if (gain <= veml6030_gains[i].gain) {
            veml6030->curr_gain_idx = i;
            break;
        }
    }

    veml6030->lux_corr_factor = veml6030_gains[veml6030->curr_gain_idx].factor 
            * veml6030_int_times[veml6030->curr_integ_time_idx].factor * VEML6030_BASE_RESOLUTION;
    return hw_VEML6030_apply_config_parameter(veml6030, VEML6030_REG_CONF_GAIN_MASK,
                            veml6030_gains[veml6030->curr_gain_idx].code);
}

float hw_VEML6030_get_gain(veml6030_inst_t* veml6030)
{
    uint16_t config = i2c_read16(veml6030->i2c_bus, veml6030->addr, VEML6030_REG_ALS_CONF);
    uint16_t code = config & VEML6030_REG_CONF_GAIN_MASK;

    veml6030->curr_gain_idx = (VEML6030_NUM_GAIN_STEPS)-1;

    for (int i=0; i < VEML6030_NUM_GAIN_STEPS; i++) {
        if (code == veml6030_gains[i].code) {
            veml6030->curr_gain_idx = i;
            break;
        }
    }

    return veml6030_gains[veml6030->curr_gain_idx].gain;
}

float hw_VEML6030_read_lux(veml6030_inst_t* veml6030) {
    uint16_t als_value;
    als_value = i2c_read16(veml6030->i2c_bus, veml6030->addr, VEML6030_REG_ALS);
    if (veml6030->auto_range && (als_value <= VEML6030_THRESH_LOW || als_value > VEML6030_THRESH_HIGH)) {
        als_value = hw_VEML6030_auto_tune(veml6030);
    }

    if (als_value == 65535) {
        return -1.0;
    } else {
        float lux = (float) als_value * veml6030->lux_corr_factor;
        if (lux > 100.0) {
            lux = 6.0135e-13 * pow(lux, 4) - 9.3924e-9 * pow(lux,3) + 8.1488e-5 * lux*lux + 1.0023 * lux; 
        }
        return lux;
    }
}
