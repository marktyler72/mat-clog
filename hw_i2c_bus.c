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

bool i2c_readfrom_mem(i2c_inst_t* i2c_bus, uint8_t addr, uint8_t* reg, size_t reg_len, 
                        uint8_t* dst, size_t dst_len)
{
    int bytes_xfer = i2c_write_blocking(i2c_bus, addr, reg, reg_len, true);  // true to keep master control of bus
    // read in one go as register addresses auto-increment
    if (bytes_xfer == reg_len) {
        bytes_xfer = i2c_read_blocking(i2c_bus, addr, dst, dst_len, false);  // false, we're done reading
        if (bytes_xfer == dst_len) return true;
    }

    return false;
}

uint8_t i2c_read8(i2c_inst_t* i2c_bus, uint8_t addr, uint8_t reg)
{
    uint8_t buf;
    if (i2c_readfrom_mem(i2c_bus, addr, &reg, sizeof reg, &buf, sizeof buf)) {
        return buf;
    }
    return 0;
}

uint8_t i2c_16_read8(i2c_inst_t* i2c_bus, uint8_t addr, uint16_t reg, bool endian_msb)
{
    uint8_t buf;
    uint8_t reg_buf[2];
    if (endian_msb) {
        reg_buf[0] = reg & 0xFF;
        reg_buf[1] = reg >> 8;
    } else {
        reg_buf[0] = reg >> 8;
        reg_buf[1] = reg & 0xFF;
    }
    if (i2c_readfrom_mem(i2c_bus, addr, reg_buf, sizeof reg_buf, &buf, sizeof buf)) {
        return buf;
    }
    return 0;
}

uint16_t i2c_read16(i2c_inst_t* i2c_bus, uint8_t addr, uint8_t reg, bool endian_msb)
{
    uint8_t buf[2];
    if (i2c_readfrom_mem(i2c_bus, addr, &reg, sizeof reg, buf, sizeof buf)) {
        if (endian_msb) {
            return (buf[1] <<8 | buf[0]);
        } else {
            return (buf[0] <<8 | buf[1]);
        }
    }
    return 0;
}

uint16_t i2c_16_read16(i2c_inst_t* i2c_bus, uint8_t addr, uint16_t reg, bool endian_msb)
{
    uint8_t reg_buf[2];
    if (endian_msb) {
        reg_buf[0] = reg & 0xFF;
        reg_buf[1] = reg >> 8;
    } else {
        reg_buf[0] = reg >> 8;
        reg_buf[1] = reg & 0xFF;
    }
    uint8_t buf[2];
    if (i2c_readfrom_mem(i2c_bus, addr, reg_buf, sizeof reg_buf, buf, sizeof buf)) {
        if (endian_msb) {
            return (buf[1] <<8 | buf[0]);
        } else {
            return (buf[0] <<8 | buf[1]);
        }
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

bool i2c_16_write8(i2c_inst_t* i2c_bus, uint8_t addr, uint16_t reg, uint8_t val, bool endian_msb)
{
    // send register number followed by its corresponding value
    uint8_t buf[3];
    if (endian_msb) {
        buf[0] = reg & 0xFF;
        buf[1] = reg >> 8;
    } else {
        buf[0] = reg >> 8;
        buf[1] = reg & 0xFF;
    }
    buf[2] = val;

    int bytes_xfer = i2c_write_blocking(i2c_bus, addr, buf, sizeof buf, false);

    if (bytes_xfer == sizeof buf) 
    {
        return true;
    }
    return false;
}

bool i2c_write16(i2c_inst_t* i2c_bus, uint8_t addr, uint8_t reg, uint16_t val, bool endian_msb)
{
    // send register number followed by its corresponding value
    uint8_t buf[3];
    buf[0] = reg;
    if (endian_msb) {
        buf[1] = val & 0xFF;
        buf[2] = val >> 8;
    } else {
        buf[1] = val >> 8;
        buf[2] = val & 0xFF;
    }
    int bytes_xfer = i2c_write_blocking(i2c_bus, addr, buf, sizeof buf, false);

    if (bytes_xfer == sizeof buf) 
    {
        return true;
    }
    return false;
}

bool i2c_16_write16(i2c_inst_t* i2c_bus, uint8_t addr, uint16_t reg, uint16_t val, bool endian_msb)
{
    // send register number followed by its corresponding value
    uint8_t buf[4];
    if (endian_msb) {
        buf[0] = reg & 0xFF;
        buf[1] = reg >> 8;
        buf[2] = val & 0xFF;
        buf[3] = val >> 8;
    } else {
        buf[0] = reg >> 8;
        buf[1] = reg & 0xFF;
        buf[2] = val >> 8;
        buf[3] = val & 0xFF;
    }
    int bytes_xfer = i2c_write_blocking(i2c_bus, addr, buf, sizeof buf, false);

    if (bytes_xfer == sizeof buf) 
    {
        return true;
    }
    return false;
}

bool i2c_writeto_mem(i2c_inst_t* i2c_bus, uint8_t addr, uint8_t* reg, size_t reg_len, 
                    uint8_t* src, size_t dst_len)
{
    int bytes_xfer = i2c_write_blocking(i2c_bus, addr, reg, reg_len, true);

    if (bytes_xfer == reg_len) {
        bytes_xfer = i2c_write_blocking(i2c_bus, addr, src, dst_len, false);
        if (bytes_xfer == dst_len) return true;
    }

    return false;
}
