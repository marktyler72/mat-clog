// Defines for a DS1307 Real Time Clock (RTC) module.
#include "hw_i2c_bus.h"

// Constants and register addresses
#define DS1307_DEFAULT_I2C_ADDR _u(0x68)

#define DS1307_REG_TIME_SEC  _u(0x00)
#define DS1307_REG_TIME_MIN  _u(0x01)
#define DS1307_REG_TIME_HR   _u(0x02)
#define DS1307_REG_TIME_DOW  _u(0x03)
#define DS1307_REG_TIME_DAY  _u(0x04)
#define DS1307_REG_TIME_MON  _u(0x05)
#define DS1307_REG_TIME_YR   _u(0x06)
#define DS1307_REG_CONTROL   _u(0x07)

typedef struct ds1307_inst {
    i2c_inst_t* i2c_bus;
    uint8_t addr;
} ds1307_inst_t;


ds1307_inst_t* hw_DS1307_init(i2c_inst_t* i2c_bus);
bool hw_DS1307_rtc_adjust(ds1307_inst_t* ds1307, datetime_t* new_rtc_time);
bool hw_DS1307_rtc_now(ds1307_inst_t* ds1307, datetime_t* rtc_time);