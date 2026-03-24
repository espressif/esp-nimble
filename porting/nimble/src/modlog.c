#include "modlog/modlog.h"

#ifdef ESP_PLATFORM
#if CONFIG_BLE_LOG_ENABLED
void MODLOG_INFO(int mod, char * msg, ...) {
    char buffer[1000];
    int len;
    va_list args;

    memset(buffer, 0, 1000);
    va_start(args, msg);
    len = sprintf(buffer, args);
    va_end(args);

    ble_log_write_hex(BLE_LOG_SRC_HOST, (uint8_t *)buffer, len);
    ble_log_flush();
}
#endif
#endif
