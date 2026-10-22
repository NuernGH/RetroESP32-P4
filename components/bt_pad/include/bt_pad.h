#pragma once
/*
 * bt_pad - Bluetooth LE gamepad support through the ESP32-C6 co-processor
 * (ESP-Hosted VHCI + NimBLE central on the P4).
 *
 * Step 1: bring-up, status and scan of nearby LE devices.
 */
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BT_PAD_IDLE = 0,    /* not started */
    BT_PAD_STARTING,    /* talking to the C6 / starting NimBLE */
    BT_PAD_READY,       /* host stack synced, can scan */
    BT_PAD_ERROR,       /* see bt_pad_status_text() */
} bt_pad_state_t;

typedef struct {
    char    name[24];
    uint8_t addr[6];
    uint8_t addr_type;
    int8_t  rssi;
    bool    hid;        /* advertises HID service or a gamepad/joystick appearance */
} bt_pad_dev_t;

/** Start the C6 link and the BLE host in a background task (idempotent). */
void bt_pad_start(void);

void bt_pad_start_after(int ms);

bt_pad_state_t bt_pad_state(void);

/** One-line human readable status (C6 firmware version, errors, scan state). */
const char *bt_pad_status_text(void);

/** Start a ~8 s scan; results are cleared first. No-op unless READY. */
void bt_pad_scan(void);
bool bt_pad_scanning(void);

/** Pair with and connect to a scanned device; remembered and reconnected automatically. */
void bt_pad_connect(const bt_pad_dev_t *dev);
bool bt_pad_connected(void);
const char *bt_pad_pad_name(void);
/** Last raw notification (hex) for diagnostics. */
const char *bt_pad_debug_text(void);
/** Forget the saved pad and its bond. */
void bt_pad_forget(void);

/** Copy up to max devices found so far, strongest HID devices first. Returns count. */
int bt_pad_get_devices(bt_pad_dev_t *out, int max);

#ifdef __cplusplus
}
#endif
