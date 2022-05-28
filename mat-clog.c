#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/util/datetime.h"
#include "hardware/rtc.h"
#include "hardware/spi.h"

#include "logger.h"
#include "hw_i2c_bus.h"
#include "hw_DS1307.h"
#include "hw_BME280.h"
#include "hw_VEML6030.h"
#include "hw_VL53L1X.h"

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

// Led define
#define PIN_LED PICO_DEFAULT_LED_PIN

void init_spi() {
    // SPI initialisation.
    spi_init(SPI_PORT, 5000*1000);  // Set to 5 MHz
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
}

void init_led() {
    // Initialise the Led.
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
}

int main() {

    stdio_init_all();
    log_to(LOGGER_DEST_STDOUT, " *** mat-clog - version 1.0 ***");

    i2c_inst_t *i2c_bus = i2c_bus_init(0, 0, 0);

    ds1307_inst_t* rtc_backup = hw_DS1307_init(i2c_bus);
    datetime_t now = {
        .year = 2022,
        .month = 05,
        .day = 21,
        .dotw = 0,
        .hour = 19,
        .min = 38,
        .sec = 30
    };
    bool status;
//    status = hw_DS1307_rtc_adjust(rtc_backup, &now);  // ** Uncomment this line to set the RTC backup to the time above.
    status = hw_DS1307_rtc_now(rtc_backup, &now);

    // Initialise the onboard real time clock and sync to the backup rtc.
    rtc_init();
    rtc_set_datetime(&now);
    sleep_ms(100);

    // Explain what we have done.
    log_to(LOGGER_DEST_STDOUT, "- System setup");
    log_to(LOGGER_DEST_STDOUT, "  -- I2C bus ready");
    log_to(LOGGER_DEST_STDOUT, "  -- RTC backup clock read");
    log_to(LOGGER_DEST_STDOUT, "  -- Internal clock set");

    // Now init the rest of the hardware
    log_to(LOGGER_DEST_STDOUT, "- Initialising hardware");
    init_led();
    log_to(LOGGER_DEST_STDOUT, "  -- Built-in LED ready");
    init_sd_card();
    log_to(LOGGER_DEST_STDOUT, "  -- SD Card mounted");


    vl53l1x_inst_t* ranger = hw_VL53L1X_init(i2c_bus);
    log_to(LOGGER_DEST_STDOUT, "  -- Distance sensor (VL53L1X) ready");

    veml6030_inst_t* als = hw_VEML6030_init(i2c_bus, VEML6030_I2C_ADDR_DEFAULT, true);
    log_to(LOGGER_DEST_STDOUT, "  -- Ambient light sensor (VEML6030) ready");

    bme280_inst_t* bme280 = hw_BME280_init(i2c_bus);
    log_to(LOGGER_DEST_STDOUT, "  -- Atmospheric sensor (BME280) ready");

    int range;
    float light, temperature, pressure, humidity;
    char results_buf[100];
    char* results_str = &results_buf[0];
    vl53l1x_result_t range_res;

    log_to(LOGGER_DEST_STDOUT, "- Starting data acquisition");
    hw_VL53L1X_start_ranging(ranger);

    sleep_ms(250); // sleep so that data polling and register update don't collide
    while (true) {
        
        gpio_put(PIN_LED, 1);
        if (hw_VL53L1X_get_distance(ranger, &range_res) == 0) {
            range = (int) range_res.distance_mm;
        } else {
            range = -1;
        }

        light = hw_VEML6030_read_lux(als);

        hw_BME280_read_values(bme280, &temperature, &pressure, &humidity);
        sprintf(results_str, "%d mm, %.1f Lux, %.1f C, %.1f hPa, %.1f %%RH", 
                    range, light, temperature, pressure, humidity);
        log_to(LOGGER_DEST_BOTH, results_str);

        // poll every 30s
        sleep_ms(200);
        gpio_put(PIN_LED, 0);
        sleep_ms(4800);
    }

    return 0;
}
