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
#include "driver/gpio.h"

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

/* Try to bring the GT911 up at one I2C address. Returns ESP_OK and sets s_tp / s_tp_io_handle. */
static esp_err_t gt911_try(i2c_master_bus_handle_t bus, uint8_t addr, int8_t int_pin)
{
    esp_lcd_panel_io_i2c_config_t io = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    io.dev_addr = addr;
    io.scl_speed_hz = 400000;   /* 100 kHz made each read ~2 ms */
    esp_err_t e = esp_lcd_new_panel_io_i2c(bus, &io, &s_tp_io_handle);
    if (e != ESP_OK) return e;

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = CONFIG_LCD_HRES,
        .y_max = CONFIG_LCD_VRES,
        .rst_gpio_num = GPIO_NUM_NC,      /* reset is done by hand below (address select) */
        .int_gpio_num = (gpio_num_t)int_pin,
        .levels = { .reset = 0, .interrupt = 0 },
        .flags = { .swap_xy = 0, .mirror_x = 0, .mirror_y = 0 },
    };
    e = esp_lcd_touch_new_i2c_gt911(s_tp_io_handle, &tp_cfg, &s_tp);
    if (e != ESP_OK) {
        esp_lcd_panel_io_del(s_tp_io_handle);
        s_tp_io_handle = NULL;
        s_tp = NULL;
    }
    return e;
}

/* GT911 power-up reset with INT held low: the chip then answers at 0x5D. */
static void gt911_reset_addr_5d(int8_t rst_pin, int8_t int_pin)
{
    gpio_config_t o = { .mode = GPIO_MODE_OUTPUT, .pin_bit_mask = (1ULL << rst_pin) | (1ULL << int_pin) };
    gpio_config(&o);
    gpio_set_level(int_pin, 0);
    gpio_set_level(rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(12));
    gpio_set_level(rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(8));
    gpio_reset_pin(int_pin);          /* back to input; the driver configures the edge interrupt */
    gpio_set_direction(int_pin, GPIO_MODE_INPUT);
    vTaskDelay(pdMS_TO_TICKS(60));
}

esp_err_t gt911_touch_init(int8_t sda_pin, int8_t scl_pin, int8_t rst_pin, int8_t int_pin)
{
    // Get the I2C bus handle (bus must be initialized beforehand)
    i2c_master_bus_handle_t i2c_handle = NULL;
    ESP_ERROR_CHECK(i2c_master_get_bus_handle(1, &i2c_handle));

    ESP_LOGI(TAG, "Initialize touch controller GT911");
    esp_err_t e = ESP_FAIL;
    bool irq_pins = (rst_pin >= 0 && int_pin >= 0);

    if (irq_pins) {   /* 1) hardware reset (INT low -> 0x5D), interrupt wired */
        gt911_reset_addr_5d(rst_pin, int_pin);
        e = gt911_try(i2c_handle, 0x5D, int_pin);
        if (e != ESP_OK) {
            ESP_LOGW(TAG, "GT911 not at 0x5D after reset (%s), trying 0x14", esp_err_to_name(e));
            e = gt911_try(i2c_handle, 0x14, int_pin);
        }
    }
    if (e != ESP_OK) {   /* 2) the original, known-good setup: no reset, no interrupt, polling */
        if (irq_pins) ESP_LOGW(TAG, "touch interrupt setup failed, falling back to polling");
        irq_pins = false;
        e = gt911_try(i2c_handle, 0x5D, -1);
        if (e != ESP_OK) e = gt911_try(i2c_handle, 0x14, -1);
    }
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "GT911 init failed (%s)", esp_err_to_name(e));
        return e;
    }

    if (irq_pins) {
        if (esp_lcd_touch_register_interrupt_callback(s_tp, gt911_int_isr) == ESP_OK) {
            s_use_irq = true;
            ESP_LOGI(TAG, "touch interrupt on GPIO%d (reset GPIO%d)", int_pin, rst_pin);
        } else {
            ESP_LOGW(TAG, "touch interrupt not available, polling");
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
