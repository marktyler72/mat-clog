// hw_DS1307.c - Functions to access and control a DS1307 Real Time Clock.

#include <stdio.h>
#include "pico/stdlib.h"
#include "hw_DS1307.h"

static ds1307_inst_t ds1307_inst;

ds1307_inst_t* hw_DS1307_init(i2c_inst_t* i2c_bus) {

    ds1307_inst_t* ds1307 = {&ds1307_inst};

    ds1307->i2c_bus = i2c_bus;
    ds1307->addr = DS1307_DEFAULT_I2C_ADDR;

    return ds1307;
}

uint8_t bcd_to_int(uint8_t bcd) {
    int units, tens;
    units = bcd & 0x0F;
    tens = bcd >> 4;
    return (tens*10 + units);
}

uint8_t int_to_bcd(uint8_t val) {
    uint8_t units, tens;

    tens = val / 10;
    units = val % 10;

    return (tens << 4 | units);
}

bool hw_DS1307_rtc_adjust(ds1307_inst_t* ds1307, datetime_t* new_time) {
    uint8_t vals[7];
    bool okay = true;

    if (new_time->year < 2000 || new_time->year > 2099) {
        okay = false;
    } else {
        vals[6] = int_to_bcd(new_time->year - 2000);
    }

    if (new_time->month < 1 || new_time->month > 12) {
        okay = false;
    } else {
        vals[5] = int_to_bcd(new_time->month);
    }

    if (new_time->day < 1 || new_time->day > 31) {
        okay = false;
    } else {
        vals[4] = int_to_bcd(new_time->day);
    }

    vals[3] = 1;

    if (new_time->hour > 23) {
        okay = false;
    } else {
        vals[2] = int_to_bcd(new_time->hour);
    }

    if (new_time->min > 59) {
        okay = false;
    } else {
        vals[1] = int_to_bcd(new_time->min);
    }

    if (new_time->sec > 59) {
        okay = false;
    } else {
        vals[0] = int_to_bcd(new_time->sec);
    }

    if (okay) {
        for (int i = 0; i< sizeof vals; i++) {
            i2c_write8(ds1307->i2c_bus, ds1307->addr, DS1307_REG_TIME_SEC+i, vals[i]);
        }
    }

    return okay;
}

bool hw_DS1307_rtc_now(ds1307_inst_t* ds1307, datetime_t* now) {
    uint8_t vals[7];
    uint8_t reg = {DS1307_REG_TIME_SEC};

    if (i2c_readfrom_mem(ds1307->i2c_bus, ds1307->addr, &reg, sizeof reg, vals, sizeof vals)) {
        now->sec = bcd_to_int(vals[0]);
        now->min = bcd_to_int(vals[1]);
        now->hour = bcd_to_int(vals[2]);
        now->dotw = 1;
        now->day = bcd_to_int(vals[4]);
        now->month = bcd_to_int(vals[5]);
        now->year = bcd_to_int(vals[6]) + 2000;
        return true;
    } else {
        return false;
    }

}