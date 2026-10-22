#include "bt_pad.h"

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_hosted.h"
#include "esp_hosted_misc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "store/config/ble_store_config.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "gamepad.h"
#include <dirent.h>
#include <sys/stat.h>
#include <stdlib.h>

void ble_store_config_init(void);
#define MAX_DEVS        24
#define SCAN_MS         8000
#define BLE_UUID_HID    0x1812
#define BLE_APPEAR_JOYSTICK 0x03C3
#define BLE_APPEAR_GAMEPAD  0x03C4

static const char *TAG = "bt_pad";

static volatile bt_pad_state_t s_state = BT_PAD_IDLE;
static volatile bool s_synced = false;
static volatile bool s_scanning = false;
static char s_status[64] = "bluetooth off";
static bt_pad_dev_t s_devs[MAX_DEVS];
static int s_ndev = 0;
static SemaphoreHandle_t s_lock;

static void set_status(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void set_status(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    vsnprintf(s_status, sizeof(s_status), fmt, ap);
    xSemaphoreGive(s_lock);
    va_end(ap);
}

static void host_task(void *arg)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void try_connect(void);
static bool load_target(void);
static bool s_have_target;

static void on_reset(int reason)
{
    s_synced = false;
    ESP_LOGW(TAG, "host reset, reason=%d", reason);
}

static void on_sync(void)
{
    {   /* own static random address: do not depend on the C6's public address */
        ble_addr_t a;
        int r = ble_hs_id_gen_rnd(1, &a);
        if (r == 0) r = ble_hs_id_set_rnd(a.val);
        ESP_LOGI(TAG, "own random address rc=%d", r);
    }
    s_synced = true;
    s_state = BT_PAD_READY;
    set_status("ready - tap to scan");
    ESP_LOGI(TAG, "host synced");
    if (!s_have_target && load_target()) s_have_target = true;
    try_connect();
}

static void add_device(const struct ble_gap_disc_desc *d)
{
    struct ble_hs_adv_fields f;
    memset(&f, 0, sizeof(f));
    ble_hs_adv_parse_fields(&f, d->data, d->length_data);

    bool hid = false;
    for (int i = 0; i < f.num_uuids16; i++)
        if (f.uuids16[i].value == BLE_UUID_HID) hid = true;
    if (f.appearance_is_present &&
        (f.appearance == BLE_APPEAR_GAMEPAD || f.appearance == BLE_APPEAR_JOYSTICK))
        hid = true;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    int idx = -1;
    for (int i = 0; i < s_ndev; i++)
        if (!memcmp(s_devs[i].addr, d->addr.val, 6)) { idx = i; break; }
    if (idx < 0 && s_ndev < MAX_DEVS) {
        idx = s_ndev++;
        memset(&s_devs[idx], 0, sizeof(s_devs[idx]));
        memcpy(s_devs[idx].addr, d->addr.val, 6);
        s_devs[idx].addr_type = d->addr.type;
    }
    if (idx >= 0) {
        s_devs[idx].rssi = d->rssi;
        if (hid) s_devs[idx].hid = true;
        if (f.name && f.name_len > 0 && s_devs[idx].name[0] == '\0') {
            int n = f.name_len < (int)sizeof(s_devs[idx].name) - 1 ? f.name_len : (int)sizeof(s_devs[idx].name) - 1;
            memcpy(s_devs[idx].name, f.name, n);
            s_devs[idx].name[n] = '\0';
        }
    }
    xSemaphoreGive(s_lock);
}

/* ---------------- connection state ---------------- */

#define MAX_FIELDS 48
#define MAX_RCHARS 8
#define MAP_MAX    512

typedef struct {
    uint8_t  rid;
    uint16_t bitoff;
    uint8_t  size;
    int32_t  lmin, lmax;
    uint16_t page, usage;
} hid_field_t;

typedef struct {
    uint16_t val_handle, cccd_handle, rref_handle;
    uint8_t  rid;
    bool     notify;
} rchar_t;

static volatile uint16_t s_conn = 0xFFFF;
static bt_pad_dev_t s_target;            /* pad we connect to / are connected to */
static bool s_have_target = false;
static volatile bool s_connected = false;
static volatile bool s_connecting = false;
static int s_enc_status = -1, s_step_fail = 0;
static bool s_retried_legacy = false;
static char s_dbg[96] = "";
static volatile uint32_t s_nnotify = 0;
const char *bt_pad_debug_text(void) { return s_dbg; }
static char s_pad_name[24];
static uint16_t s_pad_pid;

static uint16_t s_hid_start, s_hid_end;
static uint16_t s_map_handle;
static uint8_t s_map[MAP_MAX];
static int s_map_len;
static hid_field_t s_fields[MAX_FIELDS];
static int s_nfields;
static rchar_t s_rch[MAX_RCHARS];
static int s_nrch;
static int s_step;
static bool s_has_rx, s_has_ry;
static bool s_xbox_names;

static int gap_event(struct ble_gap_event *ev, void *arg);
static void try_connect(void);
static void gatt_next(void);

/* ---------------- NVS: remembered pad ---------------- */

static void save_target(void)
{
    nvs_handle_t h;
    if (nvs_open("btpad", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, "pad", &s_target, sizeof(s_target));
    nvs_commit(h);
    nvs_close(h);
}

static bool load_target(void)
{
    nvs_handle_t h;
    size_t len = sizeof(s_target);
    bool ok = false;
    if (nvs_open("btpad", NVS_READONLY, &h) == ESP_OK) {
        ok = nvs_get_blob(h, "pad", &s_target, &len) == ESP_OK && len == sizeof(s_target);
        nvs_close(h);
    }
    return ok;
}


/* ---------------- Pairing backup on the SD card ----------------
 * A full flash wipes NVS (bonds + remembered pad). Mirror the "btpad" and "nimble_bond" NVS
 * namespaces to /sd/odroid/btpad.bak after each pairing and restore them at start-up when
 * NVS has no remembered pad, so the pad survives re-flashing. File: repeated records of
 * [ns u8 len][ns][key u8 len][key][len u32][blob bytes]. */
#define BAK_PATH "/sd/odroid/btpad.bak"
static const char *const BAK_NS[] = { "btpad", "nimble_bond" };

static bool sd_ready(void)
{
    DIR *d = opendir("/sd");
    if (!d) return false;
    closedir(d);
    return true;
}

static void bak_write_all(void)
{
    if (!sd_ready()) return;
    mkdir("/sd/odroid", 0777);
    FILE *f = fopen(BAK_PATH ".tmp", "wb");
    if (!f) return;
    int nrec = 0;
    for (size_t i = 0; i < sizeof(BAK_NS) / sizeof(BAK_NS[0]); i++) {
        nvs_iterator_t it = NULL;
        if (nvs_entry_find(NVS_DEFAULT_PART_NAME, BAK_NS[i], NVS_TYPE_BLOB, &it) != ESP_OK) continue;
        while (it) {
            nvs_entry_info_t info;
            nvs_entry_info(it, &info);
            nvs_handle_t h;
            size_t len = 0;
            if (nvs_open(BAK_NS[i], NVS_READONLY, &h) == ESP_OK) {
                if (nvs_get_blob(h, info.key, NULL, &len) == ESP_OK && len > 0 && len < 4096) {
                    uint8_t *buf = malloc(len);
                    if (buf && nvs_get_blob(h, info.key, buf, &len) == ESP_OK) {
                        uint8_t nl = (uint8_t)strlen(BAK_NS[i]), kl = (uint8_t)strlen(info.key);
                        uint32_t l32 = (uint32_t)len;
                        fwrite(&nl, 1, 1, f); fwrite(BAK_NS[i], 1, nl, f);
                        fwrite(&kl, 1, 1, f); fwrite(info.key, 1, kl, f);
                        fwrite(&l32, 4, 1, f); fwrite(buf, 1, len, f);
                        nrec++;
                    }
                    free(buf);
                }
                nvs_close(h);
            }
            if (nvs_entry_next(&it) != ESP_OK) break;
        }
        nvs_release_iterator(it);
    }
    fclose(f);
    if (nrec > 0) {
        remove(BAK_PATH);
        rename(BAK_PATH ".tmp", BAK_PATH);
    } else {
        remove(BAK_PATH ".tmp");
    }
    ESP_LOGI(TAG, "pairing backed up to SD (%d records)", nrec);
}

static void bak_restore(void)
{
    FILE *f = fopen(BAK_PATH, "rb");
    if (!f) return;
    int nrec = 0;
    for (;;) {
        uint8_t nl, kl; char ns[24], key[24]; uint32_t l32;
        if (fread(&nl, 1, 1, f) != 1 || nl == 0 || nl >= sizeof(ns)) break;
        if (fread(ns, 1, nl, f) != nl) break;
        ns[nl] = 0;
        if (fread(&kl, 1, 1, f) != 1 || kl == 0 || kl >= sizeof(key)) break;
        if (fread(key, 1, kl, f) != kl) break;
        key[kl] = 0;
        if (fread(&l32, 4, 1, f) != 1 || l32 == 0 || l32 >= 4096) break;
        uint8_t *buf = malloc(l32);
        if (!buf || fread(buf, 1, l32, f) != l32) { free(buf); break; }
        nvs_handle_t h;
        if (nvs_open(ns, NVS_READWRITE, &h) == ESP_OK) {
            if (nvs_set_blob(h, key, buf, l32) == ESP_OK) { nvs_commit(h); nrec++; }
            nvs_close(h);
        }
        free(buf);
    }
    fclose(f);
    ESP_LOGI(TAG, "pairing restored from SD (%d records)", nrec);
}

static void bak_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(3000));   /* let NimBLE finish writing its keys */
    bak_write_all();
    vTaskDelete(NULL);
}

static void bak_schedule(void)
{
    xTaskCreate(bak_task, "bt_pad_bak", 4096, NULL, 3, NULL);
}

static void bak_delete(void) { remove(BAK_PATH); }

void bt_pad_forget(void)
{
    nvs_handle_t h;
    if (nvs_open("btpad", NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, "pad");
        nvs_commit(h);
        nvs_close(h);
    }
    s_have_target = false;
    if (s_connecting) ble_gap_conn_cancel();
    if (s_connected) ble_gap_terminate(s_conn, BLE_ERR_REM_USER_CONN_TERM);
    ble_store_clear();
    bak_delete();
    set_status("saved pad forgotten");
}

/* ---------------- HID report map parser ---------------- */

static void parse_report_map(const uint8_t *d, int len)
{
    s_nfields = 0;
    s_has_rx = s_has_ry = false;
    uint16_t page = 0;
    int32_t lmin = 0, lmax = 0;
    uint32_t rsize = 0, rcount = 0, rid = 0;
    uint16_t usages[16]; int nus = 0;
    uint16_t umin = 0, umax = 0; bool have_range = false;
    uint16_t bitpos[16] = {0};   /* per report id (ids >15 share slot 15) */

    int i = 0;
    while (i < len) {
        uint8_t b = d[i++];
        if (b == 0xFE) { /* long item */
            if (i >= len) break;
            i += 2 + d[i];
            continue;
        }
        int sz = b & 3; if (sz == 3) sz = 4;
        int type = (b >> 2) & 3, tag = b >> 4;
        if (i + sz > len) break;
        int32_t u = 0, sv = 0;
        for (int k = 0; k < sz; k++) u |= (uint32_t)d[i + k] << (8 * k);
        sv = u;
        if (sz == 1) sv = (int8_t)u; else if (sz == 2) sv = (int16_t)u;
        i += sz;

        if (type == 1) {            /* global */
            switch (tag) {
            case 0x0: page = u; break;
            case 0x1: lmin = sv; break;
            case 0x2: lmax = (lmin < 0 || sz == 4) ? sv : (int32_t)u; break;
            case 0x7: rsize = u; break;
            case 0x8: rid = u; break;
            case 0x9: rcount = u; break;
            }
        } else if (type == 2) {     /* local */
            switch (tag) {
            case 0x0: if (nus < 16) usages[nus++] = u; break;
            case 0x1: umin = u; have_range = true; break;
            case 0x2: umax = u; break;
            }
        } else if (type == 0) {     /* main */
            if (tag == 0x8) {       /* Input */
                int slot = rid > 15 ? 15 : rid;
                bool constant = u & 1;
                for (uint32_t k = 0; k < rcount; k++) {
                    if (!constant && s_nfields < MAX_FIELDS) {
                        uint16_t us = 0;
                        if (have_range) us = umin + k;
                        else if ((int)k < nus) us = usages[k];
                        else if (nus) us = usages[nus - 1];
                        if (us || page == 0x09) {
                            hid_field_t *f = &s_fields[s_nfields++];
                            f->rid = rid; f->bitoff = bitpos[slot] + k * rsize;
                            f->size = rsize; f->lmin = lmin; f->lmax = lmax;
                            f->page = page; f->usage = us;
                            if (page == 0x01 && us == 0x33) s_has_rx = true;
                            if (page == 0x01 && us == 0x34) s_has_ry = true;
                        }
                    }
                }
                bitpos[slot] += rcount * rsize;
            }
            if (tag == 0x8 || tag == 0x9 || tag == 0xB || tag == 0xA || tag == 0xC) {
                nus = 0; have_range = false;
            }
        }
    }
    ESP_LOGI(TAG, "report map: %d bytes, %d input fields", len, s_nfields);
    for (int o = 0; o < len; o += 24) {
        char hx[24 * 3 + 1]; int hp = 0;
        for (int k = o; k < len && k < o + 24; k++) hp += snprintf(hx + hp, sizeof(hx) - hp, "%02X ", d[k]);
        ESP_LOGI(TAG, "map[%3d] %s", o, hx);
    }
    for (int k = 0; k < s_nfields; k++)
        ESP_LOGI(TAG, "field %d: rid %u bit %u size %u page %02X usage %02X", k, (unsigned)s_fields[k].rid,
                 (unsigned)s_fields[k].bitoff, (unsigned)s_fields[k].size, s_fields[k].page, s_fields[k].usage);
}

static uint32_t get_bits(const uint8_t *d, int len, uint16_t off, uint8_t size, bool sign)
{
    uint32_t v = 0;
    for (int k = 0; k < size && k < 32; k++) {
        int bit = off + k;
        if ((bit >> 3) >= len) break;
        if (d[bit >> 3] & (1 << (bit & 7))) v |= 1u << k;
    }
    if (sign && size < 32 && (v & (1u << (size - 1)))) v |= ~0u << size;
    return v;
}

static uint32_t map_button(uint16_t u)
{
    if (s_xbox_names) {
        switch (u) {
        case 1: return GAMEPAD_BTN_A;   case 2: return GAMEPAD_BTN_B;
        case 4: return GAMEPAD_BTN_X;   case 5: return GAMEPAD_BTN_Y;
        case 7: return GAMEPAD_BTN_L1;  case 8: return GAMEPAD_BTN_R1;
        case 11: return GAMEPAD_BTN_SELECT; case 12: return GAMEPAD_BTN_START;
        case 13: return GAMEPAD_BTN_HOME;
        case 14: return GAMEPAD_BTN_L3; case 15: return GAMEPAD_BTN_R3;
        }
        return 0;
    }
    switch (u) {
    case 1: return GAMEPAD_BTN_A;  case 2: return GAMEPAD_BTN_B;
    case 3: return GAMEPAD_BTN_X;  case 4: return GAMEPAD_BTN_Y;
    case 5: return GAMEPAD_BTN_L1; case 6: return GAMEPAD_BTN_R1;
    case 7: return GAMEPAD_BTN_L2; case 8: return GAMEPAD_BTN_R2;
    case 9: return GAMEPAD_BTN_SELECT; case 10: return GAMEPAD_BTN_START;
    case 11: return GAMEPAD_BTN_L3; case 12: return GAMEPAD_BTN_R3;
    case 13: return GAMEPAD_BTN_HOME;
    }
    return 0;
}

static int scale_axis(uint32_t raw, const hid_field_t *f)
{
    int32_t r = (int32_t)raw;
    if (f->lmax == f->lmin) return 0;
    return (int)(((int64_t)(r - f->lmin) * 255) / (f->lmax - f->lmin)) - 128;
}

static int scale_trig(uint32_t raw, const hid_field_t *f)
{
    int32_t r = (int32_t)raw;
    if (f->lmax == f->lmin) return 0;
    int v = (int)(((int64_t)(r - f->lmin) * 1023) / (f->lmax - f->lmin));
    return v < 0 ? 0 : v > 1023 ? 1023 : v;
}

/* Xbox Wireless Controller (BLE), 16-byte input report, no report ID in the HoG payload:
 * 0 LX, 2 LY, 4 RX, 6 RY (u16, centre 0x8000), 8 LT, 10 RT (10 bit), 12 hat (1..8),
 * 13-14 buttons (A B - X Y - LB RB - - View Menu Xbox L3 R3), 15 Share. */
static void handle_xbox_report(const uint8_t *d, int len)
{
    if (len < 15) return;
    gamepad_state_t gs;
    memset(&gs, 0, sizeof(gs));
    gs.axis_lx = (int)(((d[0] | (d[1] << 8)) - 32768) >> 8);
    gs.axis_ly = (int)(((d[2] | (d[3] << 8)) - 32768) >> 8);
    gs.axis_rx = (int)(((d[4] | (d[5] << 8)) - 32768) >> 8);
    gs.axis_ry = (int)(((d[6] | (d[7] << 8)) - 32768) >> 8);
    gs.brake    = (d[8]  | (d[9]  << 8)) & 0x3FF;
    gs.throttle = (d[10] | (d[11] << 8)) & 0x3FF;
    static const uint8_t hat[9] = {
        0, GAMEPAD_DPAD_UP, GAMEPAD_DPAD_UP | GAMEPAD_DPAD_RIGHT, GAMEPAD_DPAD_RIGHT,
        GAMEPAD_DPAD_DOWN | GAMEPAD_DPAD_RIGHT, GAMEPAD_DPAD_DOWN,
        GAMEPAD_DPAD_DOWN | GAMEPAD_DPAD_LEFT, GAMEPAD_DPAD_LEFT, GAMEPAD_DPAD_UP | GAMEPAD_DPAD_LEFT };
    if ((d[12] & 0x0F) <= 8) gs.dpad = hat[d[12] & 0x0F];
    uint16_t b = d[13] | (d[14] << 8);
    if (b & (1 << 0))  gs.buttons |= GAMEPAD_BTN_A;
    if (b & (1 << 1))  gs.buttons |= GAMEPAD_BTN_B;
    if (b & (1 << 3))  gs.buttons |= GAMEPAD_BTN_X;
    if (b & (1 << 4))  gs.buttons |= GAMEPAD_BTN_Y;
    if (b & (1 << 6))  gs.buttons |= GAMEPAD_BTN_L1;
    if (b & (1 << 7))  gs.buttons |= GAMEPAD_BTN_R1;
    if (b & (1 << 10)) gs.buttons |= GAMEPAD_BTN_SELECT;
    if (b & (1 << 11)) gs.buttons |= GAMEPAD_BTN_START;
    if (b & (1 << 12)) gs.buttons |= GAMEPAD_BTN_HOME;
    if (b & (1 << 13)) gs.buttons |= GAMEPAD_BTN_L3;
    if (b & (1 << 14)) gs.buttons |= GAMEPAD_BTN_R3;
    if (len > 15 && (d[15] & 1)) gs.buttons |= GAMEPAD_BTN_MISC;
    if (gs.brake > 300)    gs.buttons |= GAMEPAD_BTN_L2;
    if (gs.throttle > 300) gs.buttons |= GAMEPAD_BTN_R2;
    gamepad_bt_set(&gs, GAMEPAD_BT_VID, s_pad_pid);
}

static void handle_report(uint8_t rid, const uint8_t *d, int len)
{
    if (s_xbox_names && len >= 15) { handle_xbox_report(d, len); return; }
    static gamepad_state_t gs;       /* keep state across split reports */
    for (int i = 0; i < s_nfields; i++) {
        const hid_field_t *f = &s_fields[i];
        if (f->rid != rid) continue;
        bool sign = f->lmin < 0;
        uint32_t raw = get_bits(d, len, f->bitoff, f->size, sign);
        if (f->page == 0x09) {
            uint32_t m = map_button(f->usage);
            if (m) { if (raw) gs.buttons |= m; else gs.buttons &= ~m; }
        } else if (f->page == 0x01) {
            switch (f->usage) {
            case 0x30: gs.axis_lx = scale_axis(raw, f); break;
            case 0x31: gs.axis_ly = scale_axis(raw, f); break;
            case 0x32:
                if (s_has_rx) gs.brake = scale_trig(raw, f);
                else gs.axis_rx = scale_axis(raw, f);
                break;
            case 0x33: gs.axis_rx = scale_axis(raw, f); break;
            case 0x34: gs.axis_ry = scale_axis(raw, f); break;
            case 0x35:
                if (s_has_ry) gs.throttle = scale_trig(raw, f);
                else gs.axis_ry = scale_axis(raw, f);
                break;
            case 0x39: {
                int dir = (int)raw - f->lmin;
                uint8_t dp = 0;
                if (dir >= 0 && dir <= 7) {
                    static const uint8_t t[8] = {
                        GAMEPAD_DPAD_UP, GAMEPAD_DPAD_UP | GAMEPAD_DPAD_RIGHT, GAMEPAD_DPAD_RIGHT,
                        GAMEPAD_DPAD_DOWN | GAMEPAD_DPAD_RIGHT, GAMEPAD_DPAD_DOWN,
                        GAMEPAD_DPAD_DOWN | GAMEPAD_DPAD_LEFT, GAMEPAD_DPAD_LEFT,
                        GAMEPAD_DPAD_UP | GAMEPAD_DPAD_LEFT };
                    dp = t[dir];
                }
                gs.dpad = dp;
                break; }
            case 0x90: case 0x91: case 0x92: case 0x93: {
                uint8_t bit = f->usage == 0x90 ? GAMEPAD_DPAD_UP : f->usage == 0x91 ? GAMEPAD_DPAD_DOWN :
                              f->usage == 0x92 ? GAMEPAD_DPAD_RIGHT : GAMEPAD_DPAD_LEFT;
                if (raw) gs.dpad |= bit; else gs.dpad &= ~bit;
                break; }
            }
        } else if (f->page == 0x02) {       /* simulation controls: brake / accelerator */
            if (f->usage == 0xC5) gs.brake = scale_trig(raw, f);
            if (f->usage == 0xC4) gs.throttle = scale_trig(raw, f);
        } else if (f->page == 0x0C) {       /* consumer: back / home / menu */
            uint32_t m = f->usage == 0x224 ? GAMEPAD_BTN_SELECT :
                         f->usage == 0x223 ? GAMEPAD_BTN_HOME :
                         (f->usage == 0x40 || f->usage == 0x204) ? GAMEPAD_BTN_START : 0;
            if (m) { if (raw) gs.buttons |= m; else gs.buttons &= ~m; }
        }
    }
    if (gs.brake > 300) gs.buttons |= GAMEPAD_BTN_L2; else gs.buttons &= ~GAMEPAD_BTN_L2;
    if (gs.throttle > 300) gs.buttons |= GAMEPAD_BTN_R2; else gs.buttons &= ~GAMEPAD_BTN_R2;
    gamepad_bt_set(&gs, GAMEPAD_BT_VID, s_pad_pid);
}

/* ---------------- GATT discovery chain ---------------- */

static int on_svc(uint16_t ch, const struct ble_gatt_error *e, const struct ble_gatt_svc *svc, void *a)
{
    if (e->status == 0 && svc) { s_hid_start = svc->start_handle; s_hid_end = svc->end_handle; }
    else if (e->status == BLE_HS_EDONE) { s_step++; gatt_next(); }
    else { set_status("no HID service (%d)", e->status); }
    return 0;
}

static int on_chr(uint16_t ch, const struct ble_gatt_error *e, const struct ble_gatt_chr *c, void *a)
{
    if (e->status == 0 && c) {
        uint16_t u = ble_uuid_u16(&c->uuid.u);
        if (u == 0x2A4B) s_map_handle = c->val_handle;
        else if (u == 0x2A4D && s_nrch < MAX_RCHARS && (c->properties & BLE_GATT_CHR_PROP_NOTIFY)) {
            memset(&s_rch[s_nrch], 0, sizeof(rchar_t));
            s_rch[s_nrch].val_handle = c->val_handle;
            s_rch[s_nrch].notify = true;     /* assume input until the report reference says otherwise */
            s_nrch++;
        }
    } else if (e->status == BLE_HS_EDONE) { s_step++; gatt_next(); }
    else set_status("char discovery failed (%d)", e->status);
    return 0;
}

static int on_dsc(uint16_t ch, const struct ble_gatt_error *e, uint16_t chr_val, const struct ble_gatt_dsc *d, void *a)
{
    if (e->status == 0 && d) {
        uint16_t u = ble_uuid_u16(&d->uuid.u);
        for (int i = 0; i < s_nrch; i++) {
            if (s_rch[i].val_handle != chr_val) continue;
            if (u == 0x2902) s_rch[i].cccd_handle = d->handle;
            if (u == 0x2908) s_rch[i].rref_handle = d->handle;
        }
    } else if (e->status == BLE_HS_EDONE) { s_step++; gatt_next(); }
    else set_status("descriptor discovery failed (%d)", e->status);
    return 0;
}

static int on_map(uint16_t ch, const struct ble_gatt_error *e, struct ble_gatt_attr *at, void *a)
{
    if (e->status == 0 && at && at->om) {
        int n = OS_MBUF_PKTLEN(at->om);
        if (s_map_len + n > MAP_MAX) n = MAP_MAX - s_map_len;
        if (n > 0) { os_mbuf_copydata(at->om, 0, n, s_map + s_map_len); s_map_len += n; }
    } else if (e->status == BLE_HS_EDONE || e->status == 0) {
        parse_report_map(s_map, s_map_len);
        s_step++; gatt_next();
    } else set_status("report map read failed (%d)", e->status);
    return 0;
}

static int on_rref(uint16_t ch, const struct ble_gatt_error *e, struct ble_gatt_attr *at, void *a)
{
    int idx = (int)(intptr_t)a;
    if (e->status == 0 && at && at->om && OS_MBUF_PKTLEN(at->om) >= 2) {
        uint8_t v[2];
        os_mbuf_copydata(at->om, 0, 2, v);
        s_rch[idx].rid = v[0];
        s_rch[idx].notify = (v[1] == 1);      /* 1 = input report */
        ESP_LOGI(TAG, "report char %d: handle %u id %u type %u", idx, s_rch[idx].val_handle, v[0], v[1]);
    }
    s_step++; gatt_next();
    return 0;
}

static int on_cccd(uint16_t ch, const struct ble_gatt_error *e, struct ble_gatt_attr *at, void *a)
{
    s_step++; gatt_next();
    return 0;
}

/* steps: 0 svc, 1 chrs, 2 dscs, 3 map, 4.. rref[i], then cccd[i] */
static void gatt_next(void)
{
    uint16_t ch = s_conn;
    int rc = 0;
    int base = 4;
    if (s_step == 0) {
        s_nrch = 0; s_map_len = 0; s_map_handle = 0; s_hid_start = 0;
        ble_uuid16_t u = BLE_UUID16_INIT(0x1812);
        rc = ble_gattc_disc_svc_by_uuid(ch, &u.u, on_svc, NULL);
    } else if (s_step == 1) {
        if (!s_hid_start) { set_status("pad has no HID service"); return; }
        rc = ble_gattc_disc_all_chrs(ch, s_hid_start, s_hid_end, on_chr, NULL);
    } else if (s_step == 2) {
        rc = ble_gattc_disc_all_dscs(ch, s_hid_start, s_hid_end, on_dsc, NULL);
    } else if (s_step == 3) {
        if (!s_map_handle) { set_status("pad has no report map"); return; }
        rc = ble_gattc_read_long(ch, s_map_handle, 0, on_map, NULL);
    } else if (s_step < base + s_nrch) {
        int i = s_step - base;
        if (!s_rch[i].rref_handle) { s_step++; gatt_next(); return; }
        rc = ble_gattc_read(ch, s_rch[i].rref_handle, on_rref, (void *)(intptr_t)i);
    } else if (s_step < base + 2 * s_nrch) {
        int i = s_step - base - s_nrch;
        if (!s_rch[i].notify || !s_rch[i].cccd_handle) { s_step++; gatt_next(); return; }
        uint8_t on[2] = {1, 0};
        rc = ble_gattc_write_flat(ch, s_rch[i].cccd_handle, on, 2, on_cccd, NULL);
    } else {
        s_connected = true;
        s_connecting = false;
        set_status("connected: %s", s_pad_name);
        ESP_LOGI(TAG, "pad ready: %s", s_pad_name);
        {   /* mark the pad present now: some pads only send a report when a button changes */
            gamepad_state_t idle;
            memset(&idle, 0, sizeof(idle));
            gamepad_bt_set(&idle, GAMEPAD_BT_VID, s_pad_pid);
        }
        return;
    }
    if (rc != 0) set_status("gatt step %d failed (%d)", s_step, rc);
}

/* ---------------- GAP events ---------------- */

static int gap_event(struct ble_gap_event *ev, void *arg)
{
    switch (ev->type) {
    case BLE_GAP_EVENT_DISC:
        add_device(&ev->disc);
        break;
    case BLE_GAP_EVENT_DISC_COMPLETE:
        s_scanning = false;
        set_status("scan done: %d found", s_ndev);
        try_connect();
        break;
    case BLE_GAP_EVENT_CONNECT:
        if (ev->connect.status == 0) {
            s_conn = ev->connect.conn_handle;
            set_status("pairing with %s...", s_pad_name);
            int rc = ble_gap_security_initiate(s_conn);
            if (rc != 0) { set_status("security start failed (%d)", rc); }
        } else {
            s_connecting = false;
            set_status("connect failed (%d)", ev->connect.status);
        }
        break;
    case BLE_GAP_EVENT_ENC_CHANGE:
        if (ev->enc_change.status == 0) {
            set_status("paired, reading %s...", s_pad_name);
            s_have_target = true;
            save_target();
            bak_schedule();
            {   /* ask for the shortest connection interval: lower input latency in games */
                struct ble_gap_upd_params up = {
                    .itvl_min = 6, .itvl_max = 12, .latency = 0, .supervision_timeout = 400,
                    .min_ce_len = 0, .max_ce_len = 0,
                };
                ble_gap_update_params(ev->enc_change.conn_handle, &up);
            }
            s_step = 0;
            gatt_next();
        } else {
            s_enc_status = ev->enc_change.status;
            set_status("pairing failed (%d)", ev->enc_change.status);
            ble_gap_terminate(ev->enc_change.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        }
        break;
    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        struct ble_gap_conn_desc d;
        if (ble_gap_conn_find(ev->repeat_pairing.conn_handle, &d) == 0)
            ble_store_util_delete_peer(&d.peer_id_addr);
        return BLE_GAP_REPEAT_PAIRING_RETRY; }
    case BLE_GAP_EVENT_NOTIFY_RX: {
        uint8_t buf[40];
        int n = OS_MBUF_PKTLEN(ev->notify_rx.om);
        if (n > (int)sizeof(buf)) n = sizeof(buf);
        os_mbuf_copydata(ev->notify_rx.om, 0, n, buf);
        s_nnotify++;
        {   /* log reports that differ from the previous one (capped) */
            static uint8_t last[40]; static int last_n = -1, logged = 0;
            if (logged < 150 && (n != last_n || memcmp(last, buf, n))) {
                char hx[40 * 3 + 1]; int hp = 0;
                for (int k = 0; k < n; k++) hp += snprintf(hx + hp, sizeof(hx) - hp, "%02X ", buf[k]);
                ESP_LOGI(TAG, "rx h%u l%d: %s", ev->notify_rx.attr_handle, n, hx);
                memcpy(last, buf, n); last_n = n; logged++;
            }
        }
        snprintf(s_dbg, sizeof(s_dbg), "l%d f%d x%d tail %02X%02X%02X%02X",
                 n, s_nfields, s_xbox_names,
                 n>12?buf[12]:0, n>13?buf[13]:0, n>14?buf[14]:0, n>15?buf[15]:0);
        for (int i = 0; i < s_nrch; i++)
            if (s_rch[i].val_handle == ev->notify_rx.attr_handle && s_rch[i].notify)
                handle_report(s_rch[i].rid, buf, n);
        break; }
    case BLE_GAP_EVENT_DISCONNECT: {
        bool was_ready = s_connected;
        s_connected = false;
        s_connecting = false;
        s_conn = 0xFFFF;
        gamepad_bt_set(NULL, 0, 0);
        if (was_ready) {
            set_status("pad disconnected, waiting...");
            try_connect();      /* keep waiting for the pad to come back */
        } else if (!s_retried_legacy && s_enc_status > 0) {
            /* Secure Connections pairing failed: retry once with legacy pairing */
            s_retried_legacy = true;
            ble_hs_cfg.sm_sc = 0;
            set_status("retrying legacy pairing (%d)...", s_enc_status);
            s_enc_status = -1;
            try_connect();
        } else {
            /* never got as far as a working pad: report why, do not loop */
            set_status("failed: enc %d, disc %d, step %d", s_enc_status,
                       ev->disconnect.reason, s_step);
        }
        break; }
    default:
        break;
    }
    return 0;
}

static void try_connect(void)
{
    if (!s_synced || s_scanning || s_connected || s_connecting || !s_have_target) return;
    uint8_t own_type = BLE_OWN_ADDR_RANDOM;
    ble_addr_t peer;
    peer.type = s_target.addr_type;
    memcpy(peer.val, s_target.addr, 6);
    strlcpy(s_pad_name, s_target.name[0] ? s_target.name : "pad", sizeof(s_pad_name));
    s_pad_pid = (uint16_t)(s_target.addr[0] | (s_target.addr[1] << 8)) ^
                (uint16_t)(s_target.addr[2] | (s_target.addr[3] << 8)) ^
                (uint16_t)(s_target.addr[4] | (s_target.addr[5] << 8));
    s_xbox_names = strncasecmp(s_pad_name, "Xbox", 4) == 0;
    int rc = ble_gap_connect(own_type, &peer, BLE_HS_FOREVER, NULL, gap_event, NULL);
    if (rc == 0) { s_connecting = true; set_status("waiting for %s...", s_pad_name); }
    else set_status("connect start failed (%d)", rc);
}

void bt_pad_connect(const bt_pad_dev_t *dev)
{
    if (!dev || !s_synced) return;
    if (s_scanning) ble_gap_disc_cancel();
    if (s_connecting) ble_gap_conn_cancel();
    if (s_connected) ble_gap_terminate(s_conn, BLE_ERR_REM_USER_CONN_TERM);
    s_connecting = false;
    s_scanning = false;
    s_target = *dev;
    s_have_target = true;
    s_enc_status = -1;
    s_retried_legacy = false;
    ble_hs_cfg.sm_sc = 1;
    vTaskDelay(pdMS_TO_TICKS(100));
    try_connect();
}

bool bt_pad_connected(void) { return s_connected; }
bool bt_pad_has_saved(void) { return s_have_target || load_target() ? (s_have_target = true) : false; }
const char *bt_pad_saved_name(void) { return s_target.name[0] ? s_target.name : s_pad_name; }
void bt_pad_reconnect(void)
{
    if (!s_have_target || s_connected || !s_synced) return;
    bt_pad_dev_t d = s_target;
    bt_pad_connect(&d);
}
const char *bt_pad_pad_name(void) { return s_pad_name; }

void bt_pad_scan(void)
{
    if (s_state != BT_PAD_READY || !s_synced || s_scanning) return;
    if (s_connecting) { ble_gap_conn_cancel(); s_connecting = false; vTaskDelay(pdMS_TO_TICKS(100)); }

    uint8_t own_type = BLE_OWN_ADDR_RANDOM;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_ndev = 0;
    xSemaphoreGive(s_lock);

    struct ble_gap_disc_params p = {
        .itvl = 0, .window = 0, .filter_policy = 0,
        .limited = 0, .passive = 0, .filter_duplicates = 0,
    };
    int rc = ble_gap_disc(own_type, SCAN_MS, &p, gap_event, NULL);
    for (int tries = 0; rc == BLE_HS_EBUSY && tries < 5; tries++) {
        /* a connect attempt or terminate is still in flight: cancel it and give the controller time */
        ble_gap_conn_cancel();
        ble_gap_disc_cancel();
        s_connecting = false;
        vTaskDelay(pdMS_TO_TICKS(400));
        rc = ble_gap_disc(own_type, SCAN_MS, &p, gap_event, NULL);
    }
    if (rc != 0) {
        set_status("scan failed (%d)", rc);
        ESP_LOGE(TAG, "ble_gap_disc rc=%d", rc);
        return;
    }
    s_scanning = true;
    set_status("scanning...");
}

bool bt_pad_scanning(void) { return s_scanning; }
bt_pad_state_t bt_pad_state(void) { return s_state; }

const char *bt_pad_status_text(void) { return s_status; }

static int cmp_dev(const void *a, const void *b)
{
    const bt_pad_dev_t *x = a, *y = b;
    if (x->hid != y->hid) return y->hid - x->hid;
    return y->rssi - x->rssi;
}

int bt_pad_get_devices(bt_pad_dev_t *out, int max)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    int n = s_ndev < max ? s_ndev : max;
    memcpy(out, s_devs, n * sizeof(bt_pad_dev_t));
    xSemaphoreGive(s_lock);
    qsort(out, n, sizeof(bt_pad_dev_t), cmp_dev);
    return n;
}

static void start_task(void *arg)
{
    esp_err_t nerr = nvs_flash_init();
    if (nerr == ESP_ERR_NVS_NO_FREE_PAGES || nerr == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    {   /* NVS was wiped by a flash? bring the pairing back from the SD card (wait for the card) */
        s_target.addr_type = 0;
        if (!load_target()) {
            for (int i = 0; i < 30 && !sd_ready(); i++) vTaskDelay(pdMS_TO_TICKS(200));
            bak_restore();
        }
    }
    set_status("linking to C6 radio...");
    if (esp_hosted_connect_to_slave() != 0) {
        set_status("C6 link failed");
        s_state = BT_PAD_ERROR;
        vTaskDelete(NULL);
        return;
    }

    char fwtxt[40];
    esp_hosted_coprocessor_fwver_t fw;
    int fwret = esp_hosted_get_coprocessor_fwversion(&fw);
    if (fwret == 0) {
        snprintf(fwtxt, sizeof(fwtxt), "C6 fw %u.%u.%u", (unsigned)fw.major1, (unsigned)fw.minor1, (unsigned)fw.patch1);
        set_status("%s", fwtxt);
        ESP_LOGI(TAG, "C6 fw %u.%u.%u", (unsigned)fw.major1, (unsigned)fw.minor1, (unsigned)fw.patch1);
    } else {
        snprintf(fwtxt, sizeof(fwtxt), "C6 fw unknown (%d)", fwret);
        set_status("%s", fwtxt);
        ESP_LOGW(TAG, "could not read C6 firmware version");
    }

    int r1 = esp_hosted_bt_controller_init();
    int r2 = r1 == ESP_OK ? esp_hosted_bt_controller_enable() : -1;
    /* Old C6 slave firmware (2.3.x) has no BT controller RPC (ESP_ERR_NOT_SUPPORTED, 0x106);
       its BT controller is started by the slave itself, so go straight to the host stack. */
    if (r1 == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "slave has no BT controller RPC, skipping");
        r1 = r2 = ESP_OK;
    }
    if (r1 != ESP_OK || r2 != ESP_OK) {
        set_status("%s, BT init %d enable %d", fwtxt, r1, r2);
        s_state = BT_PAD_ERROR;
        vTaskDelete(NULL);
        return;
    }

    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        set_status("%s, nimble init %d", fwtxt, (int)ret);
        s_state = BT_PAD_ERROR;
        vTaskDelete(NULL);
        return;
    }
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    set_status("%s, waiting for BLE host", fwtxt);
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_svc_gap_device_name_set("RetroP4");
    ble_store_config_init();
    nimble_port_freertos_init(host_task);
    vTaskDelete(NULL);
}

static void delayed_start_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS((int)(intptr_t)arg));
    bt_pad_start();
    vTaskDelete(NULL);
}

/* Start Bluetooth after a delay so a game can claim its memory first. */
void bt_pad_start_after(int ms)
{
    xTaskCreate(delayed_start_task, "bt_pad_delay", 3072, (void *)(intptr_t)ms, 4, NULL);
}

void bt_pad_start(void)
{
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
    if (s_state != BT_PAD_IDLE) return;
    s_state = BT_PAD_STARTING;
    xTaskCreate(start_task, "bt_pad_start", 6144, NULL, 5, NULL);
}
