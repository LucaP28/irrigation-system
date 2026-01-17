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
#include "esp_wifi.h"
#include "esp_event.h"
#include "mqtt_client.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "secrets.h"

static esp_mqtt_client_handle_t mqtt_client;

#define DHT_TYPE DHT_TYPE_AM2301
#define DHT_GPIO 23

#define RELAY_TRIGGER 0 // 0 = High-Level-Trigger : 1 = Low-Level-Trigger
#define RELAY_OFF 1     // 1 = High-Level-Trigger : 0 = Low-Level-Trigger
#define PUMP_RELAY_GPIO 32
#define PUMP_DURATION_MS 2000 // 2 Sekunden gießen
#define PUMP_WAIT_MS 20000    // 20 Sekunden warten (testweise, später um die 2 minuten)

#define BUTTON_GPIO 4
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0

// Zeitzone für Deutschland
// CET-1: Standardzeit (Winter) ist UTC + 1 Stunde
// CEST: Sommerzeit-Regel (M3.5.0 = März, letzte Woche, Sonntag)
#define TIME_ZONE "CET-1CEST,M3.5.0,M10.5.0/3"

// Diese Variablen überleben den Deep-Sleep und Stromaussetzer (solange Spannung anliegt)
static RTC_DATA_ATTR float g_lxh_total = 0;     // Licht-Summe
static RTC_DATA_ATTR long g_last_timestamp = 0; // Letzter Messzeitpunkt
static RTC_DATA_ATTR int g_last_day = -1;       // Zur Erkennung des Datumswechsels
static bool g_lxh_synced = false;               // Die gesammelten Lux/h von homeassitant ziehen, bei strom ausfall
static bool g_dry_run_alarm = false;            // Alarm wenn Wasser fehlt oder Pumpe defekt
#define MQTT_TOPIC_SYNC_LXH "home/plants/basilikum/sync_lxh"

static const char *TAG = "PLANT_MONITOR";
SSD1306_t ssd1306_dev;
bh1750_handle_t bh1750_dev;
adc_oneshot_unit_handle_t adc1_handle;
static bool mqtt_initialized = false;
static time_t last_sntp_request = 0;
static bool time_is_synchronized = false;

// Globale Grenzwerte
int32_t g_dry_val = 2300;
int32_t g_wet_val = 400;
int g_pump_threshold = 50; // Hardcoded Startwert (50%)
bool g_pump_enabled = false;

// --- NVS FUNKTIONEN ---
/*

// NVS hat keine funktion für floats, daher wird in binär gespeichert und geladen
void save_nvs_float(const char *key, float value)
{
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK)
    {
        nvs_set_blob(my_handle, key, &value, sizeof(float));
        nvs_commit(my_handle);
        nvs_close(my_handle);
        ESP_LOGI(TAG, "NVS Float gespeichert: %s = %.1f", key, value);
    }
}

// Lädt einen Float-Wert aus dem Flash
void load_nvs_float(const char *key, float *dest)
{
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READONLY, &my_handle) == ESP_OK)
    {
        size_t size = sizeof(float);
        esp_err_t err = nvs_get_blob(my_handle, key, dest, &size);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "NVS: %s nicht gefunden, nutze Standardwert.", key);
        }
        nvs_close(my_handle);
    }
}

*/
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

// --- WIFI FUNKTIONEN ---
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT verbunden! Abonniere Topics...");
        esp_mqtt_client_subscribe(client, "home/plants/basilikum/set_threshold", 0);
        esp_mqtt_client_subscribe(client, MQTT_TOPIC_SYNC_LXH, 0);
        
        // Fordere HA aktiv auf, den gespeicherten Wert zu senden
        // esp_mqtt_client_publish(client, "home/plants/basilikum/request_sync", "1", 0, 1, 0);
        break;

    case MQTT_EVENT_DATA:
        // --- Schwellenwert ---
        if (strncmp(event->topic, "home/plants/basilikum/set_threshold", event->topic_len) == 0)
        {
            char data_buf[16];
            int len = (event->data_len < 15) ? event->data_len : 15;
            memcpy(data_buf, event->data, len);
            data_buf[len] = '\0';

            int new_val = atoi(data_buf);
            if (new_val >= 0 && new_val <= 100)
            {
                g_pump_threshold = new_val;
                save_calibration("threshold", (int32_t)g_pump_threshold);
                ESP_LOGW(TAG, "MQTT: g_pump_threshold auf %d gesetzt", g_pump_threshold);
            }
        }
        // --- LXH Sync ---
        else if (strncmp(event->topic, MQTT_TOPIC_SYNC_LXH, event->topic_len) == 0)
        {
            // verarbeitet das Sync-Paket nur, wenn wir noch nicht synchronisiert sind
            if (!g_lxh_synced)
            {
                char data_buf[64];
                int len = (event->data_len < 63) ? event->data_len : 63;
                memcpy(data_buf, event->data, len);
                data_buf[len] = '\0';

                float restored_val = 0;
                int restored_day = -1;

                if (sscanf(data_buf, "{\"val\": %f, \"day\": %d}", &restored_val, &restored_day) == 2)
                {
                    time_t now;
                    time(&now);

                    // Prüfe auf valide Zeit (> Jahr 2001), da dein SNTP 2004 liefert
                    if (now > 1000000000) 
                    {
                        struct tm timeinfo;
                        localtime_r(&now, &timeinfo);

                        if (timeinfo.tm_mday == restored_day)
                        {
                            g_lxh_total = restored_val;
                            g_lxh_synced = true; 
                            ESP_LOGW(TAG, "Sync: Wert von HEUTE erfolgreich geladen (%.1f)", g_lxh_total);
                        }
                        else
                        {
                            g_lxh_total = 0;
                            g_lxh_synced = true;
                            ESP_LOGW(TAG, "Sync: Paket war von GESTERN (Tag %d, heute %d). Reset auf 0.", restored_day, timeinfo.tm_mday);
                        }
                    }
                    else 
                    {
                        // Wartet auf das nächste Paket, damit wir nicht versehentlich 0 speichern.
                        ESP_LOGI(TAG, "Sync-Paket erhalten, aber SNTP-Zeit noch nicht bereit. Ignoriere...");
                    }
                }
            }
        }
        break;

    default:
        break;
    }
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_URI,
        .credentials.username = MQTT_USER,                // Nutzer
        .credentials.authentication.password = MQTT_PASS, // Passwort
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGW(TAG, "WLAN verloren, verbinde neu...");
        esp_wifi_connect();
        // Der ESP-MQTT-Client hat eigentlich eine eingebaute Auto-Reconnect-Funktion... aber man weiß ja nie
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "IP erhalten: " IPSTR, IP2STR(&event->ip_info.ip));

        // Initialisiere den MQTT-Client nur beim allerersten Mal
        if (!mqtt_initialized)
        {
            mqtt_app_start();
            mqtt_initialized = true;
        }
    }
}

void wifi_init(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    // Registrierung der Events (WLAN-Status + IP-Status)
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);

    ESP_LOGI(TAG, "WLAN wird gestartet...");
    esp_wifi_start();
}

void mqtt_publish_data(int tds, float lux, float vpd, float moisture, float lxh, bool fert_alarm, bool dry_run_alarm)
{   
    // wartet den ersten light sync von homeassistant ab, bis es was senden darf
    if (!mqtt_initialized || !g_lxh_synced) {
        return; 
    }
    
    char payload[256]; // Buffer auf 256 erhöht, da das JSON länger wird
    snprintf(payload, sizeof(payload),
             "{\"tds\": %d, \"lux\": %.0f, \"vpd\": %.2f, \"moisture\": %.1f, \"threshold\": %d, \"lxh\": %.1f, \"fert_alarm\": %d, \"dry_run\": %d}",
             tds, lux, vpd, moisture, g_pump_threshold, lxh, (fert_alarm ? 1 : 0), (dry_run_alarm ? 1 : 0));

    esp_mqtt_client_publish(mqtt_client, "home/plants/basilikum", payload, 0, 1, 0);
    ESP_LOGI(TAG, "MQTT Sent: %s", payload);
}

// --- TIME FUNKTIONEN ---
void setup_timezone()
{
    setenv("TZ", TIME_ZONE, 1);
    tzset();

    // Aktuelle Zeit holen für den offset
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    char offset_str[10];
    // %Z = Name (CET), %z = numerischer Offset (+0100)
    strftime(offset_str, sizeof(offset_str), "%Z %z", &timeinfo);

    ESP_LOGI(TAG, "Zeitzone gesetzt auf: %s", offset_str);
}

void manage_sntp_sync()
{
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    time_t now = tv_now.tv_sec;
 
    // nutzt 1000000000 (Jahr 2001), da der Server 2004 liefert.
    bool current_sync_status = (now > 1000000000);

    // --- LOGGING & SYNC TRIGGER ---
    if (current_sync_status != time_is_synchronized)
    {
        if (current_sync_status)
        {
            ESP_LOGW(TAG, ">>> Zeit erfolgreich synchronisiert! (Unix: %ld)", now);
            
            // Sobald die Zeit da ist, fordert den Lux-Wert aktiv an, falls wir noch nicht synchronisiert sind.
            if (!g_lxh_synced && mqtt_initialized) {
                ESP_LOGW(TAG, "Zeit steht bereit. Fordere lxH Sync von HA an...");
                esp_mqtt_client_publish(mqtt_client, "home/plants/basilikum/request_sync", "1", 0, 1, 0);
            }
        }
        else
        {
            ESP_LOGE(TAG, ">>> Zeit verloren oder noch auf 1970!");
        }
        time_is_synchronized = current_sync_status;
    }

    // --- SYNC LOGIK ---
    if (!current_sync_status)
    {
        // Alle 10 Sekunden neu versuchen, bis wir eine Zeit haben
        if (now - last_sntp_request > 10 || last_sntp_request == 0)
        {
            ESP_LOGI(TAG, "Sende SNTP-Anfrage...");
            esp_sntp_stop();
            esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
            esp_sntp_setservername(0, "pool.ntp.org");
            esp_sntp_init();
            last_sntp_request = now;
        }
    }
    else
    {
        // Alle 12 Stunden Re-Sync
        if (now - last_sntp_request > 43200)
        {
            ESP_LOGI(TAG, "Geplanter 12h SNTP Re-Sync...");
            esp_sntp_stop();
            esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
            esp_sntp_setservername(0, "pool.ntp.org");
            esp_sntp_init();
            last_sntp_request = now;
        }
    }
}

void update_light_logic(float current_lux)
{
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    time_t now = tv_now.tv_sec;

    // Solange Zeit nicht synchron (Jahr < 2024), machen wir GAR NICHTS.
    if (now < 1704067200)
        return;

    struct tm timeinfo;
    localtime_r(&now, &timeinfo);

    // Beim ersten Durchlauf nach Boot den Tag setzen
    if (g_last_day == -1)
    {
        g_last_day = timeinfo.tm_mday;
        g_last_timestamp = (long)now;
        ESP_LOGI(TAG, "Licht-Logik gestartet für Tag: %d", g_last_day);
        return; // Im ersten Durchlauf noch nicht rechnen
    }

    // Zeitdifferenz berechnen
    if (g_last_timestamp > 0)
    {
        time_t diff_seconds = now - g_last_timestamp;
        if (diff_seconds > 0 && diff_seconds < 3600)
        {
            float hours = (float)diff_seconds / 3600.0;
            g_lxh_total += (current_lux * hours);
        }
    }
    g_last_timestamp = (long)now;

    // Überprüfung auf echten Datumswechsel (Mitternacht)
    if (g_last_day != timeinfo.tm_mday)
    {
        ESP_LOGW(TAG, "Mitternacht! lxH Reset (von %.1f auf 0).", g_lxh_total);
        g_lxh_total = 0;
        g_last_day = timeinfo.tm_mday; // Neuen Tag merken

        // Sofort an HA schicken, damit der Speicher dort auch auf 0 geht
        mqtt_publish_data(0, current_lux, 0, 0, g_lxh_total, false, false);
    }
}

// --- SETUP FUNKTIONEN ---
void setup_relay()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PUMP_RELAY_GPIO),
        .mode = GPIO_MODE_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(PUMP_RELAY_GPIO, RELAY_OFF);
}

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

// --- ALARM FUNKTIONEN ---
int get_dynamic_threshold(int threshold, float vpd)
{
    int vpd_threshold = threshold;
    if (vpd > 1.5)
        vpd_threshold += 15;
    else if (vpd < 0.5)
        vpd_threshold -= 10;
    return vpd_threshold;
}

void control_pump(bool threshold, float current_moisture)
{
    if (!g_pump_enabled)
    {
        gpio_set_level(PUMP_RELAY_GPIO, RELAY_OFF);
        return;
    }

    static uint32_t next_action_time = 0;
    static int state = 0;
    static float moisture_at_start = 0.0;
    static int retry_count = 0; // Zähler für erfolglose Gießversuche
    
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (now < next_action_time) return;

    switch (state) {
        case 0: // Warten
            if (threshold) {
                moisture_at_start = current_moisture;
                gpio_set_level(PUMP_RELAY_GPIO, RELAY_TRIGGER);
                state = 1;
                next_action_time = now + PUMP_DURATION_MS;
                ESP_LOGI(TAG, "Gießversuch gestartet (Versuch %d/3). Feuchte: %.1f%%", retry_count + 1, moisture_at_start);
            }
            break;

        case 1: // Gießen beendet
            gpio_set_level(PUMP_RELAY_GPIO, RELAY_OFF);
            state = 2;
            next_action_time = now + PUMP_WAIT_MS; // Sickerpause
            break;

        case 2: // Erfolgskontrolle nach Sickerpause
            if (current_moisture < (moisture_at_start + 0.5f)) {
                // Zähler hoch
                retry_count++;
                ESP_LOGW(TAG, "Keine Feuchtigkeitsänderung erkannt (%d/3)", retry_count);
                
                if (retry_count >= 3) {
                    g_dry_run_alarm = true;
                    ESP_LOGE(TAG, "!!! TROCKENLAUF-ALARM nach 3 Versuchen !!!");
                    g_pump_enabled = false; // Pumpe komplett sperren
                }
            } else {
                // Zähler zurücksetzen
                retry_count = 0;
                g_dry_run_alarm = false;
                ESP_LOGI(TAG, "Gießen erfolgreich. Zähler zurückgesetzt.");
            }
            state = 0;
            break;
    }
}
bool fertilizer_alarm(float tdsValue, float moisture)
{
    if (tdsValue > 50 && moisture > 20.0)
    {
        if (tdsValue < 500)
        {
            // Sensor hat Kontakt, aber wenig Ionen
            ssd1306_display_text(&ssd1306_dev, 6, "DUEGNER FEHLT!", 14, true); // MQTT BROKER
            return true;
        }
        else
        {
            // TDS Sensor hat Kontakt und genug Ionen
            return false;
        }
    }
    else
    {
        // TDS Sensor ist trocken oder Wasser ist noch nicht unten angekommen
        return false;
    }
}

// --------------------------- Main -----------------------------
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
    setup_relay();

    // Networking
    wifi_init(); // WLAN starten

    // Time print
    ESP_LOGI(TAG, "\n--- ZEIT-CHECK ---\n");
    setup_timezone();

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
        STATE_PUMP_TOGGLE,
        STATE_CAL_EXIT
    } system_state_t;

    system_state_t current_state = STATE_MONITOR;
    int hold_time = 0;

    // Boden feuchtigkeit linear interpolation
    float linear_wet = 0.0;

    // Luftfeuchtigkeit global
    float hum = 0;
    float temp = 0;
    static int dht_timer = 0;

    vTaskDelay(pdMS_TO_TICKS(2000)); // 2 Sekunden warten, bis alle Sensoren stabil sind

    // ------------------------- WHILE LOOP -----------------------------------

    while (1)
    {
        // --- Time Sync ---
        manage_sntp_sync();

        // ---------------------- Button Logik -----------------------------------
        int button_level = gpio_get_level(BUTTON_GPIO);

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
                    // Schaltet jetzt durch DRY -> WET -> PUMP_TOGGLE -> EXIT
                    current_state = (current_state == STATE_CAL_EXIT) ? STATE_CAL_DRY : (current_state + 1);
                }
                ssd1306_clear_screen(&ssd1306_dev, false);
            }
            hold_time = 0;
        }

        // --------------------- BUTTON HOLD LOGIK --------------------------
        if (hold_time > 12)
        {
            bool state_changed = false;

            if (current_state == STATE_CAL_DRY)
            {
                g_dry_val = read_soil_moisture();
                save_calibration("dry", g_dry_val);
                ssd1306_display_text(&ssd1306_dev, 6, "SAVED DRY!", 10, true);
                state_changed = true;
            }
            else if (current_state == STATE_CAL_WET)
            {
                g_wet_val = read_soil_moisture();
                save_calibration("wet", g_wet_val);
                ssd1306_display_text(&ssd1306_dev, 6, "SAVED NASS!", 11, true);
                state_changed = true;
            }
            else if (current_state == STATE_PUMP_TOGGLE)
            {
                g_pump_enabled = !g_pump_enabled; // Umschalten
                g_dry_run_alarm = false;          // Alarm im HA aus machen
                char *msg = g_pump_enabled ? "PUMPE AN!" : "PUMPE AUS!";
                ssd1306_display_text(&ssd1306_dev, 6, msg, strlen(msg), true);
                state_changed = true;
            }
            else if (current_state == STATE_CAL_EXIT)
            {
                char *msg = "EXIT!";
                ssd1306_display_text(&ssd1306_dev, 6, msg, strlen(msg), true);
                state_changed = true;
            }

            if (state_changed)
            {
                vTaskDelay(pdMS_TO_TICKS(1000));
                current_state = STATE_MONITOR;
                ssd1306_clear_screen(&ssd1306_dev, false);
                hold_time = 0;
            }
        }

        // ----------------------- SENSOR MESSUNG ------------------------------------
        // Sensor auslesen DHT_TYPE_DHT22
        if (dht_timer++ >= 20)
        { // Nur Alle 2 Sekunden sonst wird der sensor zu heiß
            dht_read_float_data(DHT_TYPE_AM2301, DHT_GPIO, &hum, &temp);
            dht_timer = 0;
        }

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
        linear_wet = (linear_wet * 0.9) + (percent * 0.1);

        // VPD und Alarm berechnen
        float vpd = calculate_VPD(temp, hum);
        int current_target = get_dynamic_threshold(g_pump_threshold, vpd);
        bool is_too_dry = (linear_wet < current_target); // returns true if its to dry in relation to humidity

        // Pumpe steuern
        control_pump(is_too_dry, linear_wet);

        // Licht auslesen
        float lux = 0;
        bh1750_get_data(bh1750_dev, &lux);

        // Update lux time
        update_light_logic(lux);

        // ---------------------- MQTT SEND ----------------------------------
        static int mqtt_timer = 0;
        if (mqtt_timer++ >= 20)
        {
            mqtt_publish_data(tds, lux, vpd, linear_wet, g_lxh_total, fertilizer_alarm(tds, linear_wet), g_dry_run_alarm);
            mqtt_timer = 0;
        }

        // --------------------- DISPLAY AUSGABE -----------------------------
        if (current_state == STATE_MONITOR)
        {

            // Text-Strings bauen
            char l_str[24], s_str[24], dht_str[24], vpd_str[24], tds_str[24], l_sum_str[24];
            ;
            sprintf(l_str, "Licht:  %.0f lx   ", lux);
            sprintf(s_str, "Boden:    %.1f%%   ", linear_wet);
            sprintf(dht_str, "Luft:  %.1fC %.0f%%   ", temp, hum);
            sprintf(vpd_str, "VPD:    %.2f kPa   ", vpd);
            sprintf(tds_str, "TDS:    %i ppm   ", tds);
            sprintf(l_sum_str, "Sonne:  %.0f lx/H", g_lxh_total);

            // 5. Display Ausgabe (Zeile 0 bis 7)
            // ssd1306_display_text(&ssd1306_dev, 0, "PFLANZEN-MONITOR", 16, false);
            ssd1306_display_text(&ssd1306_dev, 0, l_str, strlen(l_str), false);
            ssd1306_display_text(&ssd1306_dev, 1, s_str, strlen(s_str), false);
            ssd1306_display_text(&ssd1306_dev, 2, dht_str, strlen(dht_str), false);
            ssd1306_display_text(&ssd1306_dev, 3, vpd_str, strlen(vpd_str), false);
            ssd1306_display_text(&ssd1306_dev, 4, tds_str, strlen(tds_str), false);
            ssd1306_display_text(&ssd1306_dev, 5, l_sum_str, strlen(l_sum_str), false);

            // Alarm-Status. Invertiert, wenn "GIESSEN!" drinsteht.
            char *display_status = is_too_dry ? "GIESSEN!     " : "ALLES GUT     ";
            if (vpd > 1.5 && is_too_dry)
            {
                display_status = "TO DRY (VPD!)";
            }
            // Display Alarm
            ssd1306_display_text(&ssd1306_dev, 7, display_status, strlen(display_status), !(is_too_dry && g_pump_enabled));
        }
        else // --- SETUP MENÜ ---
        {
            ssd1306_display_text(&ssd1306_dev, 0, "---- SETUP ----", 13, false);

            char raw_str[24];
            sprintf(raw_str, "RAW: %d      ", read_soil_moisture());

            // Hier wird jetzt jeder Status korrekt angezeigt
            if (current_state == STATE_CAL_DRY)
            {
                ssd1306_display_text(&ssd1306_dev, 2, "> Set: TROCKEN   ", 15, false);
                ssd1306_display_text(&ssd1306_dev, 4, raw_str, strlen(raw_str), false);
            }
            else if (current_state == STATE_CAL_WET)
            {
                ssd1306_display_text(&ssd1306_dev, 2, "> Set: NASS      ", 15, false);
                ssd1306_display_text(&ssd1306_dev, 4, raw_str, strlen(raw_str), false);
            }
            else if (current_state == STATE_PUMP_TOGGLE)
            {
                // NEU: Zeigt an, ob die Pumpe gerade aktiviert ist oder nicht
                char p_str[24];
                sprintf(p_str, "> PUMPE: %s   ", g_pump_enabled ? "AN " : "AUS");
                ssd1306_display_text(&ssd1306_dev, 2, p_str, strlen(p_str), false);
            }
            else if (current_state == STATE_CAL_EXIT)
            {
                ssd1306_display_text(&ssd1306_dev, 2, "> ZURUECK       ", 15, false);
            }

            // Fortschrittsanzeige beim Halten (unverändert)
            if (hold_time > 9)
                ssd1306_display_text(&ssd1306_dev, 6, "HOLD: Aktion...  ", 15, false);
            else if (hold_time > 6)
                ssd1306_display_text(&ssd1306_dev, 6, "HOLD: Aktion..   ", 15, false);
            else if (hold_time > 3)
                ssd1306_display_text(&ssd1306_dev, 6, "HOLD: Aktion.    ", 15, false);
            else
                ssd1306_display_text(&ssd1306_dev, 6, "Click: Weiter    ", 15, false);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}