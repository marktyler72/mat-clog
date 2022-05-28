// Defines for hw_i2c_bus functions

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define BYTEORDER_LSB false
#define BYTEORDER_MSB true

i2c_inst_t* i2c_bus_init(int bus_no, int SDA_pin, int SCL_pin);
bool i2c_readfrom_mem(i2c_inst_t* i2c_bus, uint8_t addr, uint8_t* reg, size_t reg_len, uint8_t* dst, size_t dst_len);
bool i2c_writeto_mem(i2c_inst_t* i2c_bus, uint8_t addr, uint8_t* reg, size_t reg_len, uint8_t* src, size_t dst_len);
bool i2c_write8(i2c_inst_t* i2c_bus, uint8_t addr, uint8_t reg, uint8_t val);
bool i2c_16_write8(i2c_inst_t* i2c_bus, uint8_t addr, uint16_t reg, uint8_t val, bool endian_msb);
bool i2c_write16(i2c_inst_t* i2c_bus, uint8_t addr, uint8_t reg, uint16_t val, bool endian_msb);
bool i2c_16_write16(i2c_inst_t* i2c_bus, uint8_t addr, uint16_t reg, uint16_t val, bool endian_msb);
uint8_t i2c_read8(i2c_inst_t* i2c_bus, uint8_t addr, uint8_t reg);
uint8_t i2c_16_read8(i2c_inst_t* i2c_bus, uint8_t addr, uint16_t reg, bool endian_msb);
uint16_t i2c_read16(i2c_inst_t* i2c_bus, uint8_t addr, uint8_t reg, bool endian_msb);
uint16_t i2c_16_read16(i2c_inst_t* i2c_bus, uint8_t addr, uint16_t reg, bool endian_msb);