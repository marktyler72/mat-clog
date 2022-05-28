// Defines for the PiicoDev VL53L1X laser distance sensor.
#include "hw_i2c_bus.h"

// Constants and register addresses
#define VL53L1X_DEFAULT_I2C_ADDR _u(0x29)
#define VL53L1X_CHIPID _u(0xEACC)

#define VL53L1X_REG_RESET _u(0x0000)
#define VL53L1X_REG_I2C_ADDRESS _u(0x0001)

#define VL53L1X_REG_VHV_TIMEOUT_MACROP_LOOP_BOUND _u(0x0008)
#define VL53L1X_REG_CONFIGURE _u(0x002D)
#define VL53L1X_REG_GPIO_HV_MUX_CTRL _u(0x0030)
#define VL53L1X_REG_GPIO_TIO_HV_STATUS _u(0x0031)

#define VL53L1X_REG_PHASECAL_CONFIG__TIMEOUT_MACROP _u(0x004B)
#define VL53L1X_REG_RANGE_CONFIG__TIMEOUT_MACROP_A_HI _u(0x005E)
#define VL53L1X_REG_RANGE_CONFIG__VCSEL_PERIOD_A _u(0x0060)
#define VL53L1X_REG_RANGE_CONFIG__TIMEOUT_MACROP_B_HI _u(0x0061)
#define VL53L1X_REG_RANGE_CONFIG__VCSEL_PERIOD_B _u(0x0063)
#define VL53L1X_REG_RANGE_CONFIG__VALID_PHASE_HIGH _u(0x0069)
#define VL53L1X_REG_SD_CONFIG__WOI_SD0 _u(0x0078)
#define VL53L1X_REG_SD_CONFIG__INITIAL_PHASE_SD0 _u(0x007A)

#define VL53L1X_REG_SYSTEM_INTERRUPT_CLEAR _u(0x0086)
#define VL53L1X_REG_RANGE_START _u(0x0087)
#define VL53L1X_REG_RANGE_STATUS _u(0x0089)
#define VL53L1X_REG_CROSSTALK_CORRECTED_RANGE_MM_SD0 _u(0x0096)
#define VL53L1X_REG_SYSTEM_STATUS _u(0x00E5)
#define VL53L1X_REG_CHIPID _u(0x010F)

#define VL53L1X_DIST_MODE_SHORT   1
#define VL53L1X_DIST_MODE_LONG    2
#define VL53L1X_DIST_MODE_UNKNOWN 3

typedef struct {
    uint16_t time_ms;
    uint16_t macrop_a_hi;
    uint16_t macrop_b_hi;
} vl53l1x_timing_budget_inst;

#define VL53L1X_SHORT_RANGE_STEPS 7
static vl53l1x_timing_budget_inst vl53l1x_tb_short_dist[VL53L1X_SHORT_RANGE_STEPS] = {
                    {15, 0x001D, 0x0027}, {20, 0x0051, 0x006E}, {33, 0x00D6, 0x006E},
                    {50, 0x01AE, 0x01E8}, {100, 0x02E1, 0x0388}, {200, 0x03E1, 0x0496},
                    {500, 0x0591, 0x05C1}};

#define VL53L1X_LONG_RANGE_STEPS 6
static vl53l1x_timing_budget_inst vl53l1x_tb_long_dist[VL53L1X_LONG_RANGE_STEPS] = {
                    {20, 0x001E, 0x0022}, {33, 0x0060, 0x006E}, {50, 0x00AD, 0x00C6}, 
                    {100, 0x01CC, 0x01EA}, {200, 0x02D9, 0x02F8}, {500, 0x048F, 0x04A4}};

#define VL53L1X_NUM_CONFIG_PARAMS 91
static uint8_t vl53l1x_default_configuration[VL53L1X_NUM_CONFIG_PARAMS] = {
        0x00, // 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch
        0x00, // 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD)
        0x00, // 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD)
        0x01, // 0x30 : GPIO_HV_MUX_CTRL: set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1), use SetInterruptPolarity()
        0x02, // 0x31 : GPIO_TIO_HV_STATUS: bit 1 = interrupt depending on the polarity, use self._data_ready()
        0x00, // 0x32 : not user-modifiable (NUM)
        0x02, // 0x33 : NUM
        0x08, // 0x34 : NUM
        0x00, // 0x35 : NUM
        0x08, // 0x36 : NUM
        0x10, // 0x37 : NUM
        0x01, // 0x38 : NUM
        0x01, // 0x39 : NUM
        0x00, // 0x3a : NUM
        0x00, // 0x3b : NUM
        0x00, // 0x3c : NUM
        0x00, // 0x3d : NUM
        0xff, // 0x3e : NUM
        0x00, // 0x3f : NUM
        0x0f, // 0x40 : NUM
        0x00, // 0x41 : NUM
        0x00, // 0x42 : NUM
        0x00, // 0x43 : NUM
        0x00, // 0x44 : NUM
        0x00, // 0x45 : NUM
        0x20, // 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready , TBC
        0x0b, // 0x47 : NUM
        0x00, // 0x48 : NUM
        0x00, // 0x49 : NUM
        0x02, // 0x4a : NUM
        0x0a, // 0x4b : PHASECAL_TIMEOUT_MACROP
        0x21, // 0x4c : NUM
        0x00, // 0x4d : NUM
        0x00, // 0x4e : NUM
        0x05, // 0x4f : NUM
        0x00, // 0x50 : NUM
        0x00, // 0x51 : NUM
        0x00, // 0x52 : NUM
        0x00, // 0x53 : NUM
        0xc8, // 0x54 : NUM
        0x00, // 0x55 : NUM
        0x00, // 0x56 : NUM
        0x38, // 0x57 : NUM
        0xff, // 0x58 : NUM
        0x01, // 0x59 : NUM
        0x00, // 0x5a : NUM
        0x08, // 0x5b : NUM
        0x00, // 0x5c : NUM
        0x00, // 0x5d : NUM
        0x01, // 0x5e : TIMEOUT_MACROP_A_HI
        0xcc, // 0x5f : 
        0x0f, // 0x60 : VCSEL_PERIOD_A
        0x01, // 0x61 : TIMEOUT_MACROP_B_HI
        0xf1, // 0x62 : 
        0x0d, // 0x63 : VCSEL_PERIOD_B
        0x01, // 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), use SetSigmaThreshold(), default value 90 mm 
        0x68, // 0x65 : Sigma threshold LSB
        0x00, // 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB), use SetSignalThreshold()
        0x80, // 0x67 : Min count Rate LSB
        0x08, // 0x68 : NUM
        0xb8, // 0x69 : NUM
        0x00, // 0x6a : NUM
        0x00, // 0x6b : NUM
        0x00, // 0x6c : Intermeasurement period MSB, 32 bits register, use SetIntermeasurementInMs()
        0x00, // 0x6d : Intermeasurement period
        0x0f, // 0x6e : Intermeasurement period
        0x89, // 0x6f : Intermeasurement period LSB
        0x00, // 0x70 : NUM
        0x00, // 0x71 : NUM
        0x00, // 0x72 : distance threshold high MSB (in mm, MSB+LSB), use SetD:tanceThreshold()
        0x00, // 0x73 : distance threshold high LSB
        0x00, // 0x74 : distance threshold low MSB ( in mm, MSB+LSB), use SetD:tanceThreshold()
        0x00, // 0x75 : distance threshold low LSB
        0x00, // 0x76 : NUM
        0x01, // 0x77 : NUM
        0x0f, // 0x78 : NUM
        0x0d, // 0x79 : NUM
        0x0e, // 0x7a : NUM
        0x0e, // 0x7b : NUM
        0x00, // 0x7c : NUM
        0x00, // 0x7d : NUM
        0x02, // 0x7e : NUM
        0xc7, // 0x7f : ROI center, use SetROI()
        0xff, // 0x80 : XY ROI (X=Width, Y=Height), use SetROI()
        0x9b, // 0x81 : NUM
        0x00, // 0x82 : NUM
        0x00, // 0x83 : NUM
        0x00, // 0x84 : NUM
        0x01, // 0x85 : NUM
        0x01, // 0x86 : clear interrupt, use ClearInterrupt()
        0x00  // 0x87 : start ranging, use StartRanging() or StopRanging(), If you want an automatic start after VL53L1X_init() call, put 0x40 in location 0x87
};

typedef struct vl53l1x_inst {
    i2c_inst_t* i2c_bus;
    uint8_t addr;
    uint8_t distance_mode;
    uint16_t timing_budget_ms;
} vl53l1x_inst_t;

static const uint8_t status_rtn[24] = { 255, 255, 255, 5, 2, 4, 1, 7, 3, 0,
	255, 255, 9, 13, 255, 255, 255, 255, 10, 6,
	255, 255, 11, 12
};

typedef struct {
	uint8_t status;		
	uint16_t distance_mm;	
	uint16_t ambient;	
	uint16_t signal_per_SPAD;
	uint16_t num_SPADs;	
} vl53l1x_result_t;

vl53l1x_inst_t* hw_VL53L1X_init(i2c_inst_t* i2c_bus);
uint16_t hw_VL53L1X_get_chipid(vl53l1x_inst_t* vl53l1x);
uint8_t hw_VL53L1X_get_boot_state(vl53l1x_inst_t* vl53l1x);
void hw_VL53L1X_configure(vl53l1x_inst_t* vl53l1x);
void hw_VL53L1X_reset(vl53l1x_inst_t* vl53l1x);
uint16_t hw_VL53L1X_get_chipid(vl53l1x_inst_t* vl53l1x);
void hw_VL53L1X_start_ranging(vl53l1x_inst_t* vl53l1x);
void hw_VL53L1X_stop_ranging(vl53l1x_inst_t* vl53l1x);
void hw_VL53L1X_clear_interupt(vl53l1x_inst_t* vl53l1x);
uint8_t hw_VL53L1X_get_interrupt_polarity(vl53l1x_inst_t* vl53l1x);
void hw_VL53L1X_set_interrupt_polarity(vl53l1x_inst_t* vl53l1x, uint8_t new_polarity);
bool hw_VL53L1X_data_ready(vl53l1x_inst_t* vl53l1x);
uint8_t hw_VL53L1X_get_distance_mode(vl53l1x_inst_t* vl53l1x);
void hw_VL53L1X_set_distance_mode(vl53l1x_inst_t* vl53l1x, uint8_t new_distance_mode);
void hw_VL53L1X_set_timing_budget(vl53l1x_inst_t* vl53l1x, uint16_t new_timing_budget_ms);
uint8_t hw_VL53L1X_get_distance(vl53l1x_inst_t* vl53l1x, vl53l1x_result_t *result);