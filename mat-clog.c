#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/util/datetime.h"
#include "hardware/spi.h"
#include "hardware/rtc.h"
#include "ff.h"     // FatFs headers
#include "f_util.h"
#include "hw_config.h"

#include "hw_i2c_bus.h"
#include "hw_BME280.h"
#include "hw_VEML6030.h"

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

// Globals
sd_card_t *SD_card_ptr;

void init_spi() {
    // SPI initialisation. This example will use SPI at 5MHz.
    spi_init(SPI_PORT, 5000*1000);
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

void init_sd_card() {
    // Mount the SD card
    SD_card_ptr = sd_get_by_num(0);
    FRESULT status = f_mount(&SD_card_ptr->fatfs, SD_card_ptr->pcName, 1);
    if (FR_OK != status) panic("f_mount error: %s (%d)\n", FRESULT_str(status), status);
}

static bool open_log_file(FIL *file_ptr) {
    // Open a file for append on the SD card
    datetime_t tim = {
        .year = 2001,
        .month = 01,
        .day = 01,
        .dotw = 0,
        .hour = 0,
        .min = 0,
        .sec = 0
        };

    if (rtc_running()) {
        rtc_get_datetime(&tim);
    }

    // Construct a filename of the form /data/yyyy-mm-dd/clog_hh.txt
    // Create the appropriate directories if required.
    char filename[64];
    int n = snprintf(filename, sizeof filename, "/data");
    FRESULT status = f_mkdir(filename);
    if (status != FR_OK && status != FR_EXIST) {
        printf("f_mkdir error: %s (%d)\n", FRESULT_str(status), status);
        return false;
    }

    n += snprintf(filename + n, sizeof filename - n, "/%04d-%02d-%02d",
                  tim.year, tim.month, tim.day);
    status = f_mkdir(filename);
    if (status != FR_OK && status != FR_EXIST) {
        printf("f_mkdir error: %s (%d)\n", FRESULT_str(status), status);
        return false;
    }

    n += snprintf(filename + n, sizeof filename - n, "/clog_%02d.txt", tim.hour);
    status = f_open(file_ptr, filename, FA_OPEN_APPEND | FA_WRITE);
    if (status != FR_OK && status != FR_EXIST) {
        printf("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(status), status);
        return false;
    }
    // Put a header at the top of a new file.
    if (f_size(file_ptr) == 0) {
        if (f_printf(file_ptr, "# Timestamp        |  Temp      Press    Humidity\n") < 0) {
            printf("f_printf error\n");
            return false;
        }
    }
    return true;
}

void logger(char* string, bool log_to_file) {
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

    if (!log_to_file) {
        printf("%s| %s\n", datetime_str, string);
    } else {
        FIL file_descriptor;
        if (open_log_file(&file_descriptor)) {
            f_printf(&file_descriptor, "%s| %s\n", datetime_str, string);
            f_close(&file_descriptor);
        }
    }
}

int main() {

    stdio_init_all();
    logger(" *** mat-clog ***", false);

    // Initialise the real time clock.
    rtc_init();
    datetime_t tim = {
        .year = 2022,
        .month = 05,
        .day = 01,
        .dotw = 0,
        .hour = 22,
        .min = 18,
        .sec = 50
    };
    rtc_set_datetime(&tim);
    sleep_ms(100);

    // Now init the rest of the hardware
    logger("- Initialising hardware", false);
    init_led();
    logger(" -- Built-in LED ready", false);
    init_sd_card();
    logger(" -- SD Card mounted", false);

    i2c_inst_t *i2c_bus = i2c_bus_init(0, 0, 0);
    logger(" -- I2C bus ready", false);

    veml6030_inst_t* als = hw_VEML6030_init(i2c_bus, VEML6030_I2C_ADDR_DEFAULT, true);
    logger(" -- Ambient light sensor (VEML6030) ready", false);

    hw_BME280_init(i2c_bus);
    // retrieve fixed compensation params
    hw_BME280_calib_param params;
    hw_BME280_get_calib_params(i2c_bus, &params);
    logger(" -- Atmospheric sensor (BME280) ready", false);

    float light, temperature, pressure, humidity;
    char results_buf[100];
    char* results_str = &results_buf[0];

    logger("- Starting data acquisition", false);

    sleep_ms(250); // sleep so that data polling and register update don't collide
    while (true) {
        
        gpio_put(PIN_LED, 1);
        light = hw_VEML6030_read_lux(als);
        printf("Light @ auto range = %f\n", light);

        hw_BME280_read_values(i2c_bus, &params, &temperature, &pressure, &humidity);
        sprintf(results_str, "%.1f Lux, %.1f C, %.1f hPa, %.1f %%RH", 
                    light, temperature, pressure, humidity);
        logger(results_str, true);

        // poll every 30s
        sleep_ms(200);
        gpio_put(PIN_LED, 0);
        sleep_ms(4800);
    }

    return 0;
}
