#ifndef APP_POWER_H
#define APP_POWER_H

#include <stdbool.h>
#include "esp_err.h"

// Runtime power-save switch, one knob for everything that trades responsiveness
// for idle current so the host can A/B it (APP_CMD_SET_SYS_POWER_SAVE) without a
// reflash and turn it off when chasing a link or timing problem.
//
//   on  (default): DFS between the crystal and APP_POWER_SAVE_CPU_MHZ, automatic
//                  light sleep whenever no lock is held, BLE advertising at the
//                  low-duty interval (app_ble_spp.c).
//   off:           CPU pinned at CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ, no light sleep,
//                  NimBLE's fast advertising: the stock firmware's behaviour.
//
// What it cannot switch: BLE controller modem sleep (CONFIG_BT_LE_SLEEP_ENABLE)
// and the light-sleep flash power-down are compile-time; the latter only matters
// while light sleep is on anyway. The setting is persisted by the caller
// (main_settings.c) and applied at boot through app_power_init().

// CPU clock while power save is on. DFS only raises the clock while some lock is
// held (BLE events, UART transfers, WiFi), so this bounds the active phases
// rather than idle. 80 MHz keeps the 921600 baud link and BLE comfortably fed.
#define APP_POWER_SAVE_CPU_MHZ      80

// Create the locks and apply the initial state. Call before app_ble_start().
esp_err_t app_power_init(bool enabled);

// Switch at runtime. Restarts advertising if it is running, so the new interval
// applies at once. Does not persist the setting.
esp_err_t app_power_set_enabled(bool enabled);

bool app_power_get_enabled(void);

#endif // APP_POWER_H
