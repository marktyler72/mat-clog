// hw_i2c_bus - Functions to configure and control a shared i2c bus.
#include "hw_i2c_bus.h"
#include <stdio.h>

i2c_inst_t* i2c_bus_init(int bus_no, int SDA_pin, int SCL_pin) 
{
    i2c_inst_t *i2c_port;

    if (bus_no == 0) {
        i2c_port = i2c0;
    } else {
        i2c_port = i2c1;
    }

    // Default pins for SDA and SCL.
    int _sda_pin;
    int _scl_pin;

    if (SDA_pin == 0) {
        _sda_pin = 8;
    } else {
        _sda_pin = SDA_pin;
    }

    if (SCL_pin == 0) {
        _scl_pin = 9;
    } else {
        _scl_pin = SCL_pin;
    }
    
    // I2C Initialisation. Using it at 100KHz.
    i2c_init(i2c_port, 100*1000);
    
    gpio_set_function(_sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(_scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(_sda_pin);
    gpio_pull_up(_scl_pin);

    return i2c_port;
}

bool i2c_readfrom_mem(i2c_inst_t* i2c_bus, uint8_t addr, uint8_t reg, uint8_t* dst, size_t len)
{
    int bytes_xfer = i2c_write_blocking(i2c_bus, addr, &reg, 1, true);  // true to keep master control of bus
    // read in one go as register addresses auto-increment
    if (bytes_xfer == 1) {
        bytes_xfer = i2c_read_blocking(i2c_bus, addr, dst, len, false);  // false, we're done reading
        if (bytes_xfer == len) return true;
    }

    return false;
}

uint16_t i2c_read16(i2c_inst_t* i2c_bus, uint8_t addr, uint8_t reg)
{
    uint8_t buf[2];
    if (i2c_readfrom_mem(i2c_bus, addr, reg, buf, sizeof buf)) {
        return (buf[1] <<8 | buf[0]);
    }
    return 0;
}

bool i2c_write8(i2c_inst_t* i2c_bus, uint8_t addr, uint8_t reg, uint8_t val)
{

    // send register number followed by its corresponding value
    uint8_t buf[2] = { reg, val };
    if (i2c_write_blocking(i2c_bus, addr, buf, sizeof buf, false) == sizeof buf) 
    {
        return true;
    }
    return false;
}

bool i2c_write16(i2c_inst_t* i2c_bus, uint8_t addr, uint8_t reg, uint16_t val)
{
    // send register number followed by its corresponding value
    uint8_t buf[3] = {reg, val & 0xFF, val >> 8}; // register or command code, LSB first, MSB next
    int bytes_xfer = i2c_write_blocking(i2c_bus, addr, buf, sizeof buf, false);

    if (bytes_xfer == sizeof buf) 
    {
        return true;
    }
    return false;
}

bool i2c_writeto_mem(i2c_inst_t* i2c_bus, uint8_t addr, uint8_t reg, uint8_t* src, size_t len)
{
    int bytes_xfer = i2c_write_blocking(i2c_bus, addr, &reg, 1, true);

    if (bytes_xfer == 1) {
        bytes_xfer = i2c_write_blocking(i2c_bus, addr, src, len, false);
        if (bytes_xfer == len) return true;
    }

    return false;
}
