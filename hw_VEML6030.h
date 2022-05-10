// Defines for the PiicoDev VEML6030 ambient light sensor.
#include "hw_i2c_bus.h"

// Constants and register addresses
#define VEML6030_I2C_ADDR_DEFAULT _u(0x10)
#define VEML6030_I2C_ADDR_ALT _u(0x48)

#define VEML6030_REG_ALS_CONF _u(0x00)
#define VEML6030_REG_ALS_WH _u(0x01)
#define VEML6030_REG_ALS_WL _u(0x02)
#define VEML6030_REG_ALS _u(0x04)
#define VEML6030_REG_WHITE _u(0x05)
#define VEML6030_REG_ALS_INT _u(0x06)

// Device power control codes
#define VEML6030_ON  0
#define VEML6030_OFF 1
#define VEML6030_ONOFF_MASK _u(0xFFFE)

// Power save modes
#define VEML6030_POWER_SAVE_1 (0b00 << 1)
#define VEML6030_POWER_SAVE_2 (0b01 << 1)
#define VEML6030_POWER_SAVE_3 (0b10 << 1)
#define VEML6030_POWER_SAVE_4 (0b11 << 1)

// Power save enable
#define VEML6030_POWER_SAVE_OFF 0b0
#define VEML6030_POWER_SAVE_ON  0b1

// Configuration register masks
#define VEML6030_REG_CONF_GAIN_MASK       (0x03 << 11)
#define VEML6030_REG_CONF_INT_TIME_MASK   (0x0F << 6)
#define VEML6030_REG_CONF_INT_TIME_ENABLE (0x01 << 1)

// Integration time (ms) codes
#define VEML6030_INT_TIME_25  _u(0b1100 << 6)
#define VEML6030_INT_TIME_50  _u(0b1000 << 6)
#define VEML6030_INT_TIME_100 _u(0b0000 << 6)
#define VEML6030_INT_TIME_200 _u(0b0001 << 6)
#define VEML6030_INT_TIME_400 _u(0b0010 << 6)
#define VEML6030_INT_TIME_800 _u(0b0011 << 6)

// ALS gain codes
#define VEML6030_GAIN_x1   _u(0b0000 << 11)
#define VEML6030_GAIN_x2   _u(0b0001 << 11)
#define VEML6030_GAIN_x1_8 _u(0b0010 << 11)
#define VEML6030_GAIN_x1_4 _u(0b0011 << 11)

#define VEML6030_BASE_RESOLUTION 0.0576  // Lux per ADC unit

#define VEML6030_THRESH_LOW 200
#define VEML6030_THRESH_HIGH 7000

typedef struct veml6030_inst {
    i2c_inst_t* i2c_bus;
    uint8_t addr;
    int16_t curr_integ_time_idx;
    int16_t curr_gain_idx;
    int16_t curr_integ_time_ms;
    float lux_corr_factor;
    bool auto_range;
} veml6030_inst_t;

typedef struct {
    float gain;
    uint16_t code;
    float factor;
} veml_gain_inst;

#define VEML6030_NUM_GAIN_STEPS 4
static veml_gain_inst veml6030_gains[VEML6030_NUM_GAIN_STEPS] = {
                        {1.0/8, VEML6030_GAIN_x1_8, 8.0}, {1.0/4, VEML6030_GAIN_x1_4, 4.0}, 
                        {1.0, VEML6030_GAIN_x1, 1.0}, {2.0, VEML6030_GAIN_x2, 0.5}};

typedef struct {
    int16_t integ_time_ms;
    uint16_t code;
    float factor;
} veml_int_time_inst;

#define VEML6030_NUM_INT_TIME_STEPS 6
static veml_int_time_inst veml6030_int_times[VEML6030_NUM_INT_TIME_STEPS] = {
                        {25, VEML6030_INT_TIME_25, 4.0}, {50, VEML6030_INT_TIME_50, 2.0}, 
                        {100, VEML6030_INT_TIME_100, 1.0}, {200, VEML6030_INT_TIME_200, 0.5}, 
                        {400, VEML6030_INT_TIME_400, 0.25}, {800, VEML6030_INT_TIME_800, 0.125}};

// 
veml6030_inst_t* hw_VEML6030_init(i2c_inst_t* i2c_bus, uint8_t address, bool auto_tune);
uint16_t hw_VEML6030_auto_tune(veml6030_inst_t* veml6030);
float hw_VEML6030_read_lux(veml6030_inst_t* veml6030);
bool hw_VEML6030_set_integration_time(veml6030_inst_t* veml6030, uint16_t integ_time);
uint16_t hw_VEML6030_get_integration_time(veml6030_inst_t* veml6030);
bool hw_VEML6030_set_gain(veml6030_inst_t* veml6030, float gain);
float hw_VEML6030_get_gain(veml6030_inst_t* veml6030);