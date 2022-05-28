// logger.c - Functions to log strings to a log file and/or screen.

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/rtc.h"
#include "ff.h"     // FatFs headers
#include "f_util.h"
#include "hw_config.h"

#include "logger.h"

// Globals
sd_card_t *SD_card_ptr;

void init_sd_card(void) {
    // Mount the SD card
    SD_card_ptr = sd_get_by_num(0);
    FRESULT status = f_mount(&SD_card_ptr->fatfs, SD_card_ptr->pcName, 1);
    if (FR_OK != status) panic("?? f_mount error: %s (%d)\n", FRESULT_str(status), status);
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
        printf("?? f_mkdir error: %s (%d)\n", FRESULT_str(status), status);
        return false;
    }

    n += snprintf(filename + n, sizeof filename - n, "/%04d-%02d-%02d",
                  tim.year, tim.month, tim.day);
    status = f_mkdir(filename);
    if (status != FR_OK && status != FR_EXIST) {
        printf("?? f_mkdir error: %s (%d)\n", FRESULT_str(status), status);
        return false;
    }

    n += snprintf(filename + n, sizeof filename - n, "/clog_%02d.txt", tim.hour);
    status = f_open(file_ptr, filename, FA_OPEN_APPEND | FA_WRITE);
    if (status != FR_OK && status != FR_EXIST) {
        printf("?? f_open(%s) error: %s (%d)\n", filename, FRESULT_str(status), status);
        return false;
    }
    // Put a header at the top of a new file.
    if (f_size(file_ptr) == 0) {
        if (f_printf(file_ptr, "# Timestamp        |  Range   Light   Temp      Press    Humidity\n") < 0) {
            printf("?? f_printf error\n");
            return false;
        }
    }
    return true;
}

void log_to(uint8_t destination, char* string) {
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

    if (destination != LOGGER_DEST_FILE) {
        printf("%s| %s\n", datetime_str, string);
    } 
    
    if (destination != LOGGER_DEST_STDOUT) {
        FIL file_descriptor;
        if (open_log_file(&file_descriptor)) {
            f_printf(&file_descriptor, "%s| %s\n", datetime_str, string);
            f_close(&file_descriptor);
        }
    }
}
