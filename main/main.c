#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "bh1750.h"
#include "ssd1306.h"
#include "driver/gpio.h"

#define BUTTON_GPIO 4
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 23
#define I2C_MASTER_NUM I2C_NUM_0

static const char *TAG = "PLANT_MONITOR";
SSD1306_t ssd1306_dev;
bh1750_handle_t bh1750_dev;
adc_oneshot_unit_handle_t adc1_handle;

// Globale Kalibrierwerte
int32_t g_dry_val = 2200;
int32_t g_wet_val = 800;

// --- NVS FUNKTIONEN ---
void save_calibration(const char *key, int32_t value)
{
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK)
    {
        nvs_set_i32(my_handle, key, value);
        nvs_commit(my_handle);
        nvs_close(my_handle);
        ESP_LOGI(TAG, "Gespeichert: %s = %ld", key, value);
    }
}

void load_calibration()
{
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READONLY, &my_handle) == ESP_OK)
    {
        nvs_get_i32(my_handle, "dry", &g_dry_val);
        nvs_get_i32(my_handle, "wet", &g_wet_val);
        nvs_close(my_handle);
        ESP_LOGI(TAG, "Geladen: Dry=%ld, Wet=%ld", g_dry_val, g_wet_val);
    }
}

// --- SETUP FUNKTIONEN ---
void setup_button()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE};
    gpio_config(&io_conf);
}

void setup_soil_sensor()
{
    adc_oneshot_unit_init_cfg_t init_cfg = {.unit_id = ADC_UNIT_1};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc1_handle));
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &config));
}

int read_soil_moisture()
{
    int val;
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &val);
    return val;
}
void app_main(void)
{
    // 1. NVS, Button, Soil Sensor Init (wie gehabt)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    load_calibration();
    setup_button();
    setup_soil_sensor();

    // 2. I2C Init
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = 1,
        .scl_pullup_en = 1,
        .master.clk_speed = 100000,
    };
    i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
    i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode, 0, 0, 0);

    // 3. BH1750 Init
    bh1750_dev = bh1750_create(I2C_MASTER_NUM, BH1750_I2C_ADDRESS_DEFAULT);
    bh1750_set_measure_mode(bh1750_dev, BH1750_CONTINUE_1LX_RES);
    
    // 4. SSD1306 Init
    ssd1306_dev._address = 0x3C;
    ssd1306_init(&ssd1306_dev, 128, 64);
    ssd1306_clear_screen(&ssd1306_dev, false);
    
    typedef enum
    {
        STATE_MONITOR,
        STATE_CAL_DRY,
        STATE_CAL_WET,
        STATE_CAL_EXIT
    } system_state_t;

    system_state_t current_state = STATE_MONITOR;
    int hold_time = 0;

    while (1)
    {
        int button_level = gpio_get_level(BUTTON_GPIO);

        // Button Logik
        if (button_level == 0)
        {
            hold_time++;
        }
        else
        {
            // Kurz drücken: Menü umschalten
            if (hold_time > 1 && hold_time <= 15)
            {
                if (current_state == STATE_MONITOR)
                {
                    current_state = STATE_CAL_DRY;
                }
                else
                {
                    current_state = (current_state == STATE_CAL_EXIT) ? STATE_CAL_DRY : (current_state + 1);
                }
                ssd1306_clear_screen(&ssd1306_dev, false);
            }
            hold_time = 0;
        }

        // Aktion bei langem Druck
        if (hold_time > 20)
        {
            if (current_state == STATE_CAL_DRY)
            {
                g_dry_val = read_soil_moisture();
                save_calibration("dry", g_dry_val);
                ssd1306_display_text(&ssd1306_dev, 7, "SAVED DRY!", 10, true);
            }
            else if (current_state == STATE_CAL_WET)
            {
                g_wet_val = read_soil_moisture();
                save_calibration("wet", g_wet_val);
                ssd1306_display_text(&ssd1306_dev, 7, "SAVED NASS!", 11, true);
            }

            if (current_state != STATE_MONITOR)
            {
                vTaskDelay(pdMS_TO_TICKS(1000));
                current_state = STATE_MONITOR;
                ssd1306_clear_screen(&ssd1306_dev, false);
                hold_time = 0;
            }
        }

        // Display Ausgabe
        if (current_state == STATE_MONITOR)
        {
            int raw = read_soil_moisture();
            int percent = 0;
            if (g_dry_val != g_wet_val)
            {
                percent = (g_dry_val - raw) * 100 / (g_dry_val - g_wet_val);
            }
            if (percent < 0)
                percent = 0;
            if (percent > 100)
                percent = 100;

            float lux = 0;
            bh1750_get_data(bh1750_dev, &lux);

            char l_str[24], s_str[24];
            sprintf(l_str, "Licht: %.0f lx   ", lux);
            sprintf(s_str, "Boden: %d %%     ", percent);

            ssd1306_display_text(&ssd1306_dev, 0, "PFLANZEN-MONITOR", 16, false);
            ssd1306_display_text(&ssd1306_dev, 2, l_str, strlen(l_str), false);
            ssd1306_display_text(&ssd1306_dev, 4, s_str, strlen(s_str), false);
        }
        else
        {
            ssd1306_display_text(&ssd1306_dev, 0, "---- SETUP ----", 13, false);
            char raw_str[24];
            sprintf(raw_str, "RAW: %d      ", read_soil_moisture());
            ssd1306_display_text(&ssd1306_dev, 4, raw_str, strlen(raw_str), false);

            if (current_state == STATE_CAL_DRY)
                ssd1306_display_text(&ssd1306_dev, 2, "> Set: TROCKEN   ", 15, false);
            else if (current_state == STATE_CAL_WET)
                ssd1306_display_text(&ssd1306_dev, 2, "> Set: NASS   ", 15, false);
            else
                ssd1306_display_text(&ssd1306_dev, 2, "> ZURUECK      ", 15, false);

            if (hold_time > 17)
            {
                ssd1306_display_text(&ssd1306_dev, 6, "HOLD: Save...   ", 15, false);
            }
            else if (hold_time > 8)
            {
                ssd1306_display_text(&ssd1306_dev, 6, "HOLD: Save..    ", 15, false);
            }
            else if (hold_time > 3)
            { // Ab hier fängt er an zu zählen
                ssd1306_display_text(&ssd1306_dev, 6, "HOLD: Save.     ", 15, false);
            }
            else
            {
                ssd1306_display_text(&ssd1306_dev, 6, "Click: Weiter   ", 15, false);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}