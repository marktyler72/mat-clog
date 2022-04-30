// Defines for hw_i2c_bus functions

#include "pico/stdlib.h"
#include "hardware/i2c.h"

i2c_inst_t* i2c_bus_init(int bus_no, int SDA_pin, int SCL_pin);
bool i2c_readfrom_mem(i2c_inst_t* i2c_bus, uint8_t addr, uint8_t reg, uint8_t* dst, size_t len);
bool i2c_writeto_mem(i2c_inst_t* i2c_bus, uint8_t addr, uint8_t reg, uint8_t* src, size_t len);
bool i2c_write8(i2c_inst_t* i2c_bus, uint8_t addr, uint8_t reg, uint8_t val);