
#define LOGGER_DEST_STDOUT 1
#define LOGGER_DEST_FILE 2
#define LOGGER_DEST_BOTH 3

void init_sd_card(void);
void log_to(uint8_t destination, char* string);