#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/util/datetime.h"
#include "hardware/spi.h"
#include "hardware/rtc.h"

#include "hw_i2c_bus.h"
#include "hw_BME280.h"

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

// Led define
#define LED_PIN PICO_DEFAULT_LED_PIN

void init_spi() {
    // SPI initialisation. This example will use SPI at 1MHz.
    spi_init(SPI_PORT, 1000*1000);
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
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
}

void logger(char* string) {
    datetime_t tim;
    char datetime_buf[80];
    char* datetime_str = &datetime_buf[0];

    if (rtc_running()) {
        rtc_get_datetime(&tim);
        sprintf(datetime_str, "%d-%02.2d-%02.2d %02.2d:%02.2d:%02.2d", 
                tim.year, tim.month, tim.day, tim.hour, tim.min, tim.sec);
    } else {
        datetime_str = "                   ";
    }

    printf("%s| %s\n", datetime_str, string);
}

int main() {

    stdio_init_all();

    logger(" *** mat-clog ***");

    // Initialise the real time clock.
    rtc_init();
    datetime_t tim = {
        .year = 2022,
        .month = 04,
        .day = 26,
        .dotw = 2,
        .hour = 20,
        .min = 51,
        .sec = 30
    };
    rtc_set_datetime(&tim);
    sleep_ms(100);

    // Now init the rest of the hardware
    logger("- Initialising hardware");
    init_led();
    logger(" -- Built-in LED ready");

    i2c_inst_t *i2c_bus;
    i2c_bus = i2c_bus_init(0, 0, 0);
    logger(" -- I2C bus ready");

    hw_BME280_init(i2c_bus);
    // retrieve fixed compensation params
    hw_BME280_calib_param params;
    hw_BME280_get_calib_params(i2c_bus, &params);
    logger(" -- Atmospheric sensor (BME280) ready");

    float temperature, pressure, humidity;
    char results_buf[100];
    char* results_str = &results_buf[0];

    logger("- Starting data acquisition");

    sleep_ms(250); // sleep so that data polling and register update don't collide
    while (true) {
        
        gpio_put(LED_PIN, 1);
        hw_BME280_read_values(i2c_bus, &params, &temperature, &pressure, &humidity);
        sprintf(results_str, "%.1f C, %.1f hPa, %.1f %%RH", temperature, pressure, humidity);
        logger(results_str);

        // poll every 10s
        sleep_ms(200);
        gpio_put(LED_PIN, 0);
        sleep_ms(9800);
    }

    return 0;
}
