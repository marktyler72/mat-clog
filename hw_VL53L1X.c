// hw_VL53L1X.c - Functions to access and control the PiicoDev VL53L1X Atmospheric sensor.

#include <stdio.h>
#include "pico/stdlib.h"
#include "hw_VL53L1X.h"

static vl53l1x_inst_t vl53l1x_inst;

vl53l1x_inst_t* hw_VL53L1X_init(i2c_inst_t* i2c_bus) {

    vl53l1x_inst_t* vl53l1x = {&vl53l1x_inst};

    vl53l1x->i2c_bus = i2c_bus;
    vl53l1x->addr = VL53L1X_DEFAULT_I2C_ADDR;
    vl53l1x->timing_budget_ms = 200;

    // Reset the device and check the chip ID.
    hw_VL53L1X_reset(vl53l1x);

    if (hw_VL53L1X_get_chipid(vl53l1x) != VL53L1X_CHIPID) {
      printf("?? Error reading VL53L1X chip id \n");
    }    

    // Wait for the chip to boot.
    while (hw_VL53L1X_get_boot_state(vl53l1x) == 0) {
        sleep_ms(2);
    }

    // Initialise the sensor and set default ranging parameters
    hw_VL53L1X_configure(vl53l1x);
    
    hw_VL53L1X_set_interrupt_polarity(vl53l1x, 1);
    hw_VL53L1X_set_distance_mode(vl53l1x, VL53L1X_DIST_MODE_LONG);

    hw_VL53L1X_start_ranging(vl53l1x);
    while (hw_VL53L1X_data_ready(vl53l1x) == false) {
        //printf("spin \n");
        sleep_ms(100);
    }
    hw_VL53L1X_clear_interupt(vl53l1x);
    hw_VL53L1X_stop_ranging(vl53l1x);

    i2c_16_write8(vl53l1x->i2c_bus, vl53l1x->addr, VL53L1X_REG_VHV_TIMEOUT_MACROP_LOOP_BOUND, 0x09, BYTEORDER_LSB);
    i2c_16_write8(vl53l1x->i2c_bus, vl53l1x->addr, 0x0B, 0, BYTEORDER_LSB);  // Start VHV from previous buferature

    return vl53l1x;
}

void hw_VL53L1X_configure(vl53l1x_inst_t* vl53l1x) {
    uint16_t reg = {VL53L1X_REG_CONFIGURE};
    for (uint16_t i = 0; i < VL53L1X_NUM_CONFIG_PARAMS; i++) {
        i2c_16_write8(vl53l1x->i2c_bus, vl53l1x->addr, reg, vl53l1x_default_configuration[i], BYTEORDER_LSB);
        reg++;
    }
}

void hw_VL53L1X_reset(vl53l1x_inst_t* vl53l1x) {
    i2c_16_write8(vl53l1x->i2c_bus, vl53l1x->addr, VL53L1X_REG_RESET, 0x00, BYTEORDER_LSB);
    sleep_ms(100);
    i2c_16_write8(vl53l1x->i2c_bus, vl53l1x->addr, VL53L1X_REG_RESET, 0x01, BYTEORDER_LSB);
    sleep_ms(1);
}

uint16_t hw_VL53L1X_get_chipid(vl53l1x_inst_t* vl53l1x) {
    uint16_t reg = {VL53L1X_REG_CHIPID};
    return i2c_16_read16(vl53l1x->i2c_bus, vl53l1x->addr, reg, BYTEORDER_LSB);
}

uint8_t hw_VL53L1X_get_boot_state(vl53l1x_inst_t* vl53l1x) {
    uint16_t reg = VL53L1X_REG_SYSTEM_STATUS;
    uint8_t status = i2c_16_read8(vl53l1x->i2c_bus, vl53l1x->addr, reg, BYTEORDER_LSB);
    return status;
}

void hw_VL53L1X_start_ranging(vl53l1x_inst_t* vl53l1x) {
    i2c_16_write8(vl53l1x->i2c_bus, vl53l1x->addr, VL53L1X_REG_RANGE_START, 0x40, BYTEORDER_LSB);
}

void hw_VL53L1X_stop_ranging(vl53l1x_inst_t* vl53l1x) {
    i2c_16_write8(vl53l1x->i2c_bus, vl53l1x->addr, VL53L1X_REG_RANGE_START, 0x00, BYTEORDER_LSB);
}

void hw_VL53L1X_clear_interupt(vl53l1x_inst_t* vl53l1x) {
    i2c_16_write8(vl53l1x->i2c_bus, vl53l1x->addr, VL53L1X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01, BYTEORDER_LSB);
}

uint8_t hw_VL53L1X_get_interrupt_polarity(vl53l1x_inst_t* vl53l1x) {
    uint8_t polarity = i2c_16_read8(vl53l1x->i2c_bus, vl53l1x->addr, VL53L1X_REG_GPIO_HV_MUX_CTRL, BYTEORDER_LSB);
    polarity = polarity & 0x10;
    polarity = !(polarity >>4);
    return polarity;
}

void hw_VL53L1X_set_interrupt_polarity(vl53l1x_inst_t* vl53l1x, uint8_t new_polarity) {
    uint8_t val = i2c_16_read8(vl53l1x->i2c_bus, vl53l1x->addr, VL53L1X_REG_GPIO_HV_MUX_CTRL, BYTEORDER_LSB);
    val = val & 0xEF;
    val = val | (!(new_polarity & 1)) << 4;
    i2c_16_write8(vl53l1x->i2c_bus, vl53l1x->addr, VL53L1X_REG_GPIO_HV_MUX_CTRL, val, BYTEORDER_LSB);
}

bool hw_VL53L1X_data_ready(vl53l1x_inst_t* vl53l1x) {
    uint8_t polarity = hw_VL53L1X_get_interrupt_polarity(vl53l1x);
    uint8_t status = i2c_16_read8(vl53l1x->i2c_bus, vl53l1x->addr, VL53L1X_REG_GPIO_TIO_HV_STATUS, BYTEORDER_LSB);
    if ((status & 0x01) 
            == polarity) {
        return true;
    } else {
        return false;
    }
}

uint8_t hw_VL53L1X_get_distance_mode(vl53l1x_inst_t* vl53l1x) {
    uint8_t val = i2c_16_read8(vl53l1x->i2c_bus, vl53l1x->addr, VL53L1X_REG_PHASECAL_CONFIG__TIMEOUT_MACROP, BYTEORDER_LSB);
    switch (val) {
        case 0x14:
            return VL53L1X_DIST_MODE_SHORT;
        case 0x0A: 
            return VL53L1X_DIST_MODE_LONG;
        default:
            return VL53L1X_DIST_MODE_UNKNOWN;
        }
}

void hw_VL53L1X_set_distance_mode(vl53l1x_inst_t* vl53l1x, uint8_t new_distance_mode) {

    vl53l1x->distance_mode = new_distance_mode;

    switch (new_distance_mode) {
        case VL53L1X_DIST_MODE_SHORT:
            i2c_16_write8(vl53l1x->i2c_bus, vl53l1x->addr, VL53L1X_REG_PHASECAL_CONFIG__TIMEOUT_MACROP, 0x14, BYTEORDER_LSB);
            i2c_16_write8(vl53l1x->i2c_bus, vl53l1x->addr, VL53L1X_REG_RANGE_CONFIG__VCSEL_PERIOD_A, 0x07, BYTEORDER_LSB);
            i2c_16_write8(vl53l1x->i2c_bus, vl53l1x->addr, VL53L1X_REG_RANGE_CONFIG__VCSEL_PERIOD_B, 0x05, BYTEORDER_LSB);
            i2c_16_write8(vl53l1x->i2c_bus, vl53l1x->addr, VL53L1X_REG_RANGE_CONFIG__VALID_PHASE_HIGH, 0x38, BYTEORDER_LSB);
            i2c_16_write16(vl53l1x->i2c_bus, vl53l1x->addr, VL53L1X_REG_SD_CONFIG__WOI_SD0, 0x0705, BYTEORDER_LSB);
            i2c_16_write16(vl53l1x->i2c_bus, vl53l1x->addr, VL53L1X_REG_SD_CONFIG__INITIAL_PHASE_SD0, 0x0606, BYTEORDER_LSB);
            break;
        case VL53L1X_DIST_MODE_LONG:
            i2c_16_write8(vl53l1x->i2c_bus, vl53l1x->addr, VL53L1X_REG_PHASECAL_CONFIG__TIMEOUT_MACROP, 0x0A, BYTEORDER_LSB);
            i2c_16_write8(vl53l1x->i2c_bus, vl53l1x->addr, VL53L1X_REG_RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F, BYTEORDER_LSB);
            i2c_16_write8(vl53l1x->i2c_bus, vl53l1x->addr, VL53L1X_REG_RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D, BYTEORDER_LSB);
            i2c_16_write8(vl53l1x->i2c_bus, vl53l1x->addr, VL53L1X_REG_RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8, BYTEORDER_LSB);
            i2c_16_write16(vl53l1x->i2c_bus, vl53l1x->addr, VL53L1X_REG_SD_CONFIG__WOI_SD0, 0x0F0D, BYTEORDER_LSB);
            i2c_16_write16(vl53l1x->i2c_bus, vl53l1x->addr, VL53L1X_REG_SD_CONFIG__INITIAL_PHASE_SD0, 0x0E0E, BYTEORDER_LSB);
            if (vl53l1x->timing_budget_ms == 15) {  // Can't have a 15ms timing budget for long distance mode.
                vl53l1x->timing_budget_ms = 30;
            }
            break;
        default:
            return;
        }

    hw_VL53L1X_set_timing_budget(vl53l1x, vl53l1x->timing_budget_ms);
}

void hw_VL53L1X_set_timing_budget(vl53l1x_inst_t* vl53l1x, uint16_t new_timing_budget_ms) {
    vl53l1x_timing_budget_inst* timing_parameters;
    int steps;

    switch (vl53l1x->distance_mode) {
        case VL53L1X_DIST_MODE_SHORT:
            timing_parameters = &vl53l1x_tb_short_dist[0];
            steps = VL53L1X_SHORT_RANGE_STEPS;
            break;
        case VL53L1X_DIST_MODE_LONG:
            timing_parameters = &vl53l1x_tb_long_dist[0];
            steps = VL53L1X_LONG_RANGE_STEPS;
            break;
        default:
            return;
        }

    uint16_t macrop_a_hi, macrop_b_hi;
    bool found = false;

    for (int i=0; i<steps; i++) {
        if (timing_parameters[i].time_ms == new_timing_budget_ms) {
            macrop_a_hi = timing_parameters[i].macrop_a_hi;
            macrop_b_hi = timing_parameters[i].macrop_b_hi;
            found = true;
        }
    }

    if (found) {
        i2c_16_write16(vl53l1x->i2c_bus, vl53l1x->addr, VL53L1X_REG_RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                    macrop_a_hi, BYTEORDER_LSB);
        i2c_16_write16(vl53l1x->i2c_bus, vl53l1x->addr, VL53L1X_REG_RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                    macrop_b_hi, BYTEORDER_LSB);
        vl53l1x->timing_budget_ms = new_timing_budget_ms;
    }
}

uint8_t hw_VL53L1X_get_distance(vl53l1x_inst_t* vl53l1x, vl53l1x_result_t *result) {
    int spins = 0;

    while (!hw_VL53L1X_data_ready(vl53l1x)) {
        sleep_ms(10);
        spins++;
        if (spins > 100) {      // Give up after a while.
            result->status = 255;
            result->distance_mm = 0;
            return 255;
        }
    }

    uint8_t reg[2] = {VL53L1X_REG_RANGE_STATUS >> 8, VL53L1X_REG_RANGE_STATUS & 0xFF};
    uint8_t buf[17];
    i2c_readfrom_mem(vl53l1x->i2c_bus, vl53l1x->addr, reg, sizeof reg, buf, sizeof buf);
    // uint8_t status = i2c_16_read8(vl53l1x->i2c_bus, vl53l1x->addr, VL53L1X_REG_RANGE_STATUS, BYTEORDER_LSB);
    // uint16_t range_mm = i2c_16_read16(vl53l1x->i2c_bus, vl53l1x->addr, 
    //                         VL53L1X_REG_CROSSTALK_CORRECTED_RANGE_MM_SD0, BYTEORDER_LSB);

   uint8_t status = buf[0] & 0x1F;
    // status = status & 0x1F;
	if (status < 24)
		status = status_rtn[status];
    // result->distance_mm = range_mm;
	result->status = status;
	result->ambient = (buf[7] << 8 | buf[8]) * 8;
	result->num_SPADs = buf[3];
	result->signal_per_SPAD = (buf[15] << 8 | buf[16]) * 8;
	result->distance_mm = buf[13] << 8 | buf[14];

    hw_VL53L1X_clear_interupt(vl53l1x);

    return status;
}