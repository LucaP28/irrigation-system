#include <stdio.h>
#include <string.h>
#include <math.h>
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
#include "dht.h"
#include <time.h>
#include <sys/time.h>

#define DHT_GPIO 23
#define DHT_TYPE DHT_TYPE_AM2301

#define BUTTON_GPIO 4
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0

// Diese Variablen überleben den Deep-Sleep und Stromaussetzer (solange Spannung anliegt)
static RTC_DATA_ATTR float g_lxh_total = 0;     // Licht-Summe
static RTC_DATA_ATTR long g_last_timestamp = 0; // Letzter Messzeitpunkt
static RTC_DATA_ATTR int g_last_day = -1;       // Zur Erkennung des Datumswechsels

static const char *TAG = "PLANT_MONITOR";
SSD1306_t ssd1306_dev;
bh1750_handle_t bh1750_dev;
adc_oneshot_unit_handle_t adc1_handle;

// Globale Kalibrierwerte
int32_t g_dry_val = 2300;
int32_t g_wet_val = 400;

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

// -- TIME FUNKTIONEN ---
void update_light_logic(float current_lux)
{
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);

    // WICHTIG: Benutze time_t statt long
    time_t now = tv_now.tv_sec;

    // 1. Zeitdifferenz berechnen
    if (g_last_timestamp > 0)
    {
        time_t diff_seconds = now - g_last_timestamp;

        if (diff_seconds > 0 && diff_seconds < 3600)
        {
            float hours = (float)diff_seconds / 3600.0;
            g_lxh_total += (current_lux * hours);
        }
    }
    g_last_timestamp = (long)now; // Hier casten wir zurück für den RTC Speicher

    // 2. Überprüfung auf Datumswechsel
    struct tm timeinfo;
    // Jetzt passt der Typ für localtime_r
    localtime_r(&now, &timeinfo);

    if (g_last_day != -1 && g_last_day != timeinfo.tm_mday)
    {
        ESP_LOGI(TAG, "Neuer Tag erkannt! Gestern: %.1f lxH", g_lxh_total);
        g_lxh_total = 0;
    }
    g_last_day = timeinfo.tm_mday;
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
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &config)); // soil sensor gpio 34
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_7, &config)); // tds sensor gpio 35
}

int read_soil_moisture()
{
    int val;
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &val);
    return val;
}

int read_tds_raw()
{
    int val;
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_7, &val);
    return val;
}

float get_tds_value(int raw_adc, float air_temp)
{
    // Das Wasser im Untertopf ist ca. 3°C kühler als die Luft
    float water_temp = air_temp - 3.0f;

    // Spannung berechnen (ESP32: 3.3V / 12 bit)
    float voltage = raw_adc * (3.3f / 4095.0f);

    // Temperatur-Korrektur auf Basis der Wassertemperatur
    float compensationCoefficient = 1.0 + 0.02 * (water_temp - 25.0);
    float compensationVoltage = voltage / compensationCoefficient;

    /*
    TDS Polynom
    Konvertiert Spannung in TDS (ppm).
    Die Koeffizienten basieren auf einer empirischen Polynom-Regression 3. Grades
    Standardmodell für kostengünstige Leitfähigkeitssensoren.
    Modell-Gleichung: y = 133.42x^3 - 255.86x^2 + 857.39x
    */
    float tdsValue = (133.42 * pow(compensationVoltage, 3) - 255.86 * pow(compensationVoltage, 2) + 857.39 * compensationVoltage) * 0.5;
    return tdsValue;
}

// --- VPD BERECHNUNG ---
float calculate_VPD(float temp, float hum)
{
    if (hum < 0.01)
        hum = 0.01;                                             // Division durch 0 verhindern
    float svp = 0.61078 * exp((17.27 * temp) / (temp + 237.3)); // Sättigungsdampfdruck (SVP) in kPa
    float avp = svp * (hum / 100.0);                            // Aktueller Dampfdruck (AVP)
    return svp - avp;
}

// --- ALARM ---
void check_plant_alarm(float soil_pct, float vpd, char *status_out)
{
    float threshold = 60.0; // Basis-Schwelle

    if (vpd > 1.5)
    {
        threshold = 75.0; // Luft trocken -> früher gießen
        strcpy(status_out, "DURSTIG (VPD!)");
    }
    else if (vpd < 0.5)
    {
        threshold = 50.0; // Luft feucht -> später gießen (Schutz vor Fäule)
        strcpy(status_out, "OK (FEUCHT)");
    }
    else
    {
        strcpy(status_out, soil_pct < threshold ? "GIESSEN!" : "ALLES GUT");
    }

    if (soil_pct < threshold)
    {
        // ESP_LOGW(TAG, "ALARM: Boden %.1f%% zu trocken für VPD %.2f", soil_pct, vpd);
    }
}

// --- Main ---
void app_main(void)
{
    // NVS, Button, Soil Sensor Init
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

    // Time print
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    printf("\n--- ZEIT-CHECK ---\n");
    // Monat +1, Jahr +1900
    printf("Datum: %02d.%02d.%04d\n", timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900);
    printf("Uhrzeit: %02d:%02d:%02d\n", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    printf("------------------\n\n");

    // I2C Init
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

    // BH1750 Init
    bh1750_dev = bh1750_create(I2C_MASTER_NUM, BH1750_I2C_ADDRESS_DEFAULT);
    bh1750_set_measure_mode(bh1750_dev, BH1750_CONTINUE_1LX_RES);

    // SSD1306 Init
    ssd1306_dev._address = 0x3C;
    ssd1306_init(&ssd1306_dev, 128, 64);
    ssd1306_clear_screen(&ssd1306_dev, false);

    // Menu state
    typedef enum
    {
        STATE_MONITOR,
        STATE_CAL_DRY,
        STATE_CAL_WET,
        STATE_CAL_EXIT
    } system_state_t;

    system_state_t current_state = STATE_MONITOR;
    int hold_time = 0;

    // Boden feuchtigkeit linear interpolation
    float linear_percent = 0.0;

    // Luftfeuchtigkeit global
    float hum = 0;
    float temp = 0;
    static int dht_timer = 0;

    vTaskDelay(pdMS_TO_TICKS(2000)); // 2 Sekunden warten, bis alle Sensoren stabil sind

    while (1)
    {
        // Sensor auslesen DHT_TYPE_DHT22
        if (dht_timer++ >= 20)
        { // Alle 2 Sekunden
            dht_read_float_data(DHT_TYPE_AM2301, DHT_GPIO, &hum, &temp);
            dht_timer = 0;
        }

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

        // Lang drücken: save
        if (hold_time > 12)
        {
            if (current_state == STATE_CAL_DRY)
            {
                g_dry_val = read_soil_moisture();
                save_calibration("dry", g_dry_val);
                ssd1306_display_text(&ssd1306_dev, 6, "SAVED DRY!", 10, true);
            }
            else if (current_state == STATE_CAL_WET)
            {
                g_wet_val = read_soil_moisture();
                save_calibration("wet", g_wet_val);
                ssd1306_display_text(&ssd1306_dev, 6, "SAVED NASS!", 11, true);
            }

            if (current_state != STATE_MONITOR)
            {
                vTaskDelay(pdMS_TO_TICKS(1000));
                current_state = STATE_MONITOR;
                ssd1306_clear_screen(&ssd1306_dev, false);
                hold_time = 0;
            }
        }

        // --- DISPLAY AUSGABE ---
        if (current_state == STATE_MONITOR)
        {
            // Bodenfeuchtigkeit berechnen
            int raw_soil = read_soil_moisture();
            float percent = 0.0;

            if (g_dry_val != g_wet_val)
            {
                percent = (float)(g_dry_val - raw_soil) * 100.0 / (float)(g_dry_val - g_wet_val);
            }
            if (percent < 0)
                percent = 0;
            if (percent > 100)
                percent = 100;

            // TDS auswerten
            int raw_tds = read_tds_raw();
            int tds = get_tds_value(raw_tds, temp);

            // Glättung des Werts
            linear_percent = (linear_percent * 0.9) + (percent * 0.1);

            // VPD und Alarm berechnen
            float vpd = calculate_VPD(temp, hum);
            char status_str[20];
            check_plant_alarm(linear_percent, vpd, status_str);

            // Licht auslesen
            float lux = 0;
            bh1750_get_data(bh1750_dev, &lux);

            // Update lux time
            update_light_logic(lux);

            // Text-Strings bauen
            char l_str[24], s_str[24], dht_str[24], vpd_str[24], tds_str[24], l_sum_str[24];
            ;
            sprintf(l_str, "Licht: %.0f lx   ", lux);
            sprintf(s_str, "Boden: %.1f%%   ", linear_percent);
            sprintf(dht_str, "Luft: %.1fC %.0f%%   ", temp, hum);
            sprintf(vpd_str, "VPD: %.2f kPa   ", vpd);
            sprintf(tds_str, "TDS: %i ppm   ", tds);
            sprintf(l_sum_str, "Sonne: %.0f lx/H", g_lxh_total);

            // 5. Display Ausgabe (Zeile 0 bis 7)
            ssd1306_display_text(&ssd1306_dev, 0, "PFLANZEN-MONITOR", 16, false);
            ssd1306_display_text(&ssd1306_dev, 1, l_str, strlen(l_str), false);
            ssd1306_display_text(&ssd1306_dev, 2, s_str, strlen(s_str), false);
            ssd1306_display_text(&ssd1306_dev, 3, dht_str, strlen(dht_str), false);
            ssd1306_display_text(&ssd1306_dev, 4, vpd_str, strlen(vpd_str), false);
            ssd1306_display_text(&ssd1306_dev, 5, tds_str, strlen(tds_str), false);
            ssd1306_display_text(&ssd1306_dev, 6, l_sum_str, strlen(l_sum_str), false);

            // Alarm-Status. Invertiert, wenn "GIESSEN!" drinsteht.
            bool is_alarm = (strcmp(status_str, "GIESSEN!") == 0);
            ssd1306_display_text(&ssd1306_dev, 7, status_str, strlen(status_str), is_alarm);
        }
        else // --- SETUP MENÜ ---
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

            // Fortschrittsanzeige beim Halten
            if (hold_time > 9)
                ssd1306_display_text(&ssd1306_dev, 6, "HOLD: Save...   ", 15, false);
            else if (hold_time > 6)
                ssd1306_display_text(&ssd1306_dev, 6, "HOLD: Save..    ", 15, false);
            else if (hold_time > 3)
                ssd1306_display_text(&ssd1306_dev, 6, "HOLD: Save.     ", 15, false);
            else
                ssd1306_display_text(&ssd1306_dev, 6, "Click: Weiter   ", 15, false);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}