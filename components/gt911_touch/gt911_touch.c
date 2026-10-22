/*
 * GT911 touch wrapper - ported from Arduino C++ to ESP-IDF C
 */
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "esp_lcd_touch_gt911.h"
#include "gt911_touch.h"
#include "esp_attr.h"
#include "esp_timer.h"

#define CONFIG_LCD_HRES 480
#define CONFIG_LCD_VRES 800

static const char *TAG = "gt911_touch";

static esp_lcd_touch_handle_t s_tp = NULL;
static esp_lcd_panel_io_handle_t s_tp_io_handle = NULL;

/* Interrupt-driven reads: the GT911 pulls INT on every new report. get_xy() only touches the
 * I2C bus when INT fired, while a finger is down, or as a 100 ms safety net. */
static volatile bool s_use_irq = false;
static volatile bool s_irq_pending = false;
static bool s_finger_down = false;
static int64_t s_last_read_us = 0;

static void IRAM_ATTR gt911_int_isr(esp_lcd_touch_handle_t tp)
{
    s_irq_pending = true;
}

esp_err_t gt911_touch_init(int8_t sda_pin, int8_t scl_pin, int8_t rst_pin, int8_t int_pin)
{
    // Get the I2C bus handle (bus must be initialized beforehand)
    i2c_master_bus_handle_t i2c_handle = NULL;
    ESP_ERROR_CHECK(i2c_master_get_bus_handle(1, &i2c_handle));

    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    tp_io_config.scl_speed_hz = 400000;   /* 100 kHz made each read ~2 ms */
    ESP_LOGI(TAG, "Initialize touch IO (I2C)");
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_handle, &tp_io_config, &s_tp_io_handle));

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = CONFIG_LCD_HRES,
        .y_max = CONFIG_LCD_VRES,
        .rst_gpio_num = (gpio_num_t)rst_pin,
        .int_gpio_num = (gpio_num_t)int_pin,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };

    ESP_LOGI(TAG, "Initialize touch controller GT911");
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(s_tp_io_handle, &tp_cfg, &s_tp));

    if (int_pin >= 0) {
        esp_err_t e = esp_lcd_touch_register_interrupt_callback(s_tp, gt911_int_isr);
        if (e == ESP_OK) {
            s_use_irq = true;
            ESP_LOGI(TAG, "touch interrupt on GPIO%d (reset GPIO%d)", int_pin, rst_pin);
        } else {
            ESP_LOGW(TAG, "touch interrupt not available (%s), polling", esp_err_to_name(e));
        }
    }

    return ESP_OK;
}

bool gt911_touch_get_xy(uint16_t *x, uint16_t *y)
{
    if (!s_tp) return false;

    uint16_t touch_strength[1];
    uint8_t touch_cnt = 0;

    int64_t now = esp_timer_get_time();
    if (s_use_irq && !s_irq_pending && !s_finger_down && now - s_last_read_us < 100000)
        return false;   /* nothing new from the controller */
    s_irq_pending = false;
    s_last_read_us = now;

    esp_lcd_touch_read_data(s_tp);
    bool touched = esp_lcd_touch_get_coordinates(s_tp, x, y, touch_strength, &touch_cnt, 1);
    s_finger_down = touched;

    return touched;
}
