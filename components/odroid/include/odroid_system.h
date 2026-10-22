/*
 * Odroid System Compatibility Layer for ESP32-P4
 *
 * System initialization and OTA application switching stubs.
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the system (I2C, PPA, LCD, Touch, Audio, Gamepad).
 * This sets up all hardware peripherals for the ESP32-P4 platform.
 */
void odroid_system_init(void);

/**
 * @brief Set the OTA application slot to boot on next restart.
 * On the P4, this is a stub that logs the requested slot.
 * Future: will implement actual emulator switching.
 *
 * @param slot OTA slot number (1-10 = emulator, 0 = launcher)
 */
void odroid_system_application_set(int slot);

/**
 * @brief Get the shared I2C master bus handle (created during system init).
 */
void *odroid_system_get_i2c_bus(void);

/**
 * @brief Enter deep sleep (stub on P4).
 */
void odroid_system_sleep(void);

/**
 * @brief Set the indicator LED (stub on P4 — no LED).
 * @param value 0=off, 1=on
 */
void odroid_system_led_set(int value);

#define ODROID_RESTART_MAGIC 0x52535452u
#define ODROID_TRAIL_LINES 10
typedef struct { uint32_t magic; uint32_t intentional; char task[16]; uint32_t n; char line[ODROID_TRAIL_LINES][34]; } odroid_restart_info_t;
/* Fixed LP-RAM address (not a linker symbol) so the launcher and every app see the same struct. */
#define odroid_restart_info (*(odroid_restart_info_t *)0x5010E000)
void odroid_restart_mark_intentional(void);
/* Breadcrumb trail kept in RTC RAM across the restart; the launcher shows it (no serial log on this board). */
void odroid_trail(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

#ifdef __cplusplus
}
#endif
