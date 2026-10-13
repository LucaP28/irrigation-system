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
#include "driver/gpio.h"
#include "dht.h"
#include <time.h>
#include <sys/time.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "mqtt_client.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "esp_sleep.h"
#include "secrets.h"

static esp_mqtt_client_handle_t mqtt_client;

#define DHT_TYPE DHT_TYPE_AM2301
#define DHT_GPIO 23

#define RELAY_TRIGGER 0 // 1 = High-Level-Trigger : 0 = Low-Level-Trigger
#define RELAY_OFF 1     // 0 = High-Level-Trigger : 1 = Low-Level-Trigger
#define PUMP_RELAY_GPIO 32
#define PUMP_DURATION_MS 10000 // 10 Sekunden gießen

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
static RTC_DATA_ATTR bool g_dry_run_alarm = false;    // Alarm wenn Wasser fehlt oder Pumpe defekt
static RTC_DATA_ATTR int g_retry_count = 0;           // Zähler für erfolglose Gießversuche
static RTC_DATA_ATTR float g_moisture_at_start = 0.0; // Feuchtigkeit vor dem Gießen
static RTC_DATA_ATTR bool g_is_soaking = false;       // State ob gerade in der Sickerpause nach dem Gießen

static bool g_lxh_synced = false;               // Die gesammelten Lux/h von homeassitant ziehen, bei strom ausfall
#define MQTT_TOPIC_SYNC_LXH "home/plants/basilikum/sync_lxh"

static const char *TAG = "PLANT_MONITOR";
bh1750_handle_t bh1750_dev;
adc_oneshot_unit_handle_t adc1_handle;
static bool mqtt_initialized = false;
static time_t last_sntp_request = 0;
static bool time_is_synchronized = false;
int g_last_raw_soil = 0;

// Globale Grenzwerte
int32_t g_dry_val = 2300;
int32_t g_wet_val = 400;
int32_t g_pump_threshold = 50; // Hardcoded Startwert (50%)
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
        nvs_get_i32(my_handle, "threshold", &g_pump_threshold);
        nvs_close(my_handle);
        ESP_LOGI(TAG, "Geladen: Dry=%ld, Wet=%ld, Pump-Threshold=%ld", g_dry_val, g_wet_val, g_pump_threshold);
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
        esp_mqtt_client_subscribe(client, "home/plants/basilikum/set_pump_enabled", 0);
        esp_mqtt_client_subscribe(client, MQTT_TOPIC_SYNC_LXH, 0);
        
        // get pump state retained from HA
        esp_mqtt_client_publish(client, "home/plants/basilikum/state_pump_enabled", 
                       g_pump_enabled ? "1" : "0", 0, 1, 1);
        
        // NEU: Fordere den Tages-Lichtwert an, sobald wir verbunden sind
        esp_mqtt_client_publish(client, "home/plants/basilikum/request_sync", "1", 0, 1, 0);
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
                if (new_val != g_pump_threshold) {
                    g_pump_threshold = new_val;
                    save_calibration("threshold", g_pump_threshold);
                }
                ESP_LOGW(TAG, "MQTT: g_pump_threshold auf %d gesetzt", g_pump_threshold);
            }
        }
        // --- Pumpe ON/OFF Switch ---
        else if (strncmp(event->topic, "home/plants/basilikum/set_pump_enabled", event->topic_len) == 0)
        {
            char data_buf[8];
            int len = (event->data_len < 7) ? event->data_len : 7;
            memcpy(data_buf, event->data, len);
            data_buf[len] = '\0';

            if (data_buf[0] == '1') {
                g_pump_enabled = true;
                g_dry_run_alarm = false;
                ESP_LOGW(TAG, "MQTT: Pumpe AUTOMATIK AKTIVIERT");
            } else {
                g_pump_enabled = false;
                // WICHTIG: Wenn Automatik aus, Pumpe sofort sicherheitshalber stoppen
                gpio_set_level(PUMP_RELAY_GPIO, RELAY_OFF); 
                ESP_LOGW(TAG, "MQTT: Pumpe AUTOMATIK DEAKTIVIERT");
            }
            
            // Status sofort an HA zurückmelden, damit der Schalter dort synchron bleibt
            esp_mqtt_client_publish(client, "home/plants/basilikum/state_pump_enabled", 
                                   g_pump_enabled ? "1" : "0", 0, 1, 1);
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

void mqtt_publish_data(float lux, float vpd, float moisture, float lxh, bool dry_run_alarm)
{   
    // wartet den ersten light sync von homeassistant ab, bis es was senden darf
    if (!mqtt_initialized || !g_lxh_synced) {
        return; 
    }
    
    char payload[256]; // Buffer auf 256 erhöht, da das JSON länger wird
    snprintf(payload, sizeof(payload),
             "{\"lux\": %.0f, \"vpd\": %.2f, \"moisture\": %.1f, \"threshold\": %ld, \"lxh\": %.1f, \"dry_run\": %d}",
             lux, vpd, moisture, g_pump_threshold, lxh, (dry_run_alarm ? 1 : 0));

    esp_mqtt_client_publish(mqtt_client, "home/plants/basilikum", payload, 0, 1, 0);
    ESP_LOGI(TAG, "MQTT Sent: %s", payload);
}

void mqtt_publish_debug_data(float temp, float hum, float lux, float linear_wet, float vpd)
{   
    if (!mqtt_initialized) {
        return; 
    }
    
    char payload[512]; 
    snprintf(payload, sizeof(payload), 
             "{\"raw_soil\": %d, \"wet_val\": %ld, \"dry_val\": %ld, \"temp\": %.1f, \"hum\": %.1f, "
             "\"lux\": %.0f, \"linear_wet\": %.1f, \"vpd\": %.2f, \"lxh_total\": %.1f, "
             "\"pump_enabled\": %d, \"threshold\": %ld, \"dry_run_alarm\": %d, "
             "\"retry_count\": %d, \"is_soaking\": %d}",
             g_last_raw_soil, g_wet_val, g_dry_val, temp, hum, 
             lux, linear_wet, vpd, g_lxh_total, 
             g_pump_enabled ? 1 : 0, g_pump_threshold, g_dry_run_alarm ? 1 : 0, 
             g_retry_count, g_is_soaking ? 1 : 0);

    esp_mqtt_client_publish(mqtt_client, "home/plants/basilikum/debug", payload, 0, 1, 0);
    ESP_LOGI(TAG, "MQTT Debug Sent: %s", payload);
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

    // --- LOGGING ---
    if (current_sync_status != time_is_synchronized)
    {
        if (current_sync_status)
        {
            ESP_LOGW(TAG, ">>> Zeit erfolgreich synchronisiert! (Unix: %ld)", now);
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
    }
}

// --- SETUP FUNKTIONEN ---
void setup_relay()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PUMP_RELAY_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(PUMP_RELAY_GPIO, RELAY_OFF);
}

void setup_soil_sensor()
{
    adc_oneshot_unit_init_cfg_t init_cfg = {.unit_id = ADC_UNIT_1};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc1_handle));
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &config)); // soil sensor gpio 34
}

float get_smoothed_moisture()
{
    // 1. Dem Sensor kurz Zeit geben, sein Feld aufzubauen
    vTaskDelay(pdMS_TO_TICKS(500)); 

    // 2. Dummy-Reads (Kondensator aufladen)
    int dummy;
    for(int i = 0; i < 10; i++) {
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &dummy);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // 3. 10 echte Werte in ein Array einlesen
    int readings[10];
    for(int i = 0; i < 10; i++) {
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &readings[i]);
        vTaskDelay(pdMS_TO_TICKS(50)); 
    }
    
    // 4. Array sortieren (Bubble Sort)
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9 - i; j++) {
            if (readings[j] > readings[j + 1]) {
                int temp = readings[j];
                readings[j] = readings[j + 1];
                readings[j + 1] = temp;
            }
        }
    }
    
    // 5. Die 2 niedrigsten und 2 höchsten Ausreißer ignorieren (gestutztes Mittel)
    int sum = 0;
    for (int i = 2; i < 8; i++) {
        sum += readings[i];
    }
    int raw_soil = sum / 6;
    g_last_raw_soil = raw_soil;  //debug value

    // 6. Normale Prozentrechnung
    float percent = 0.0;
    if (g_dry_val != g_wet_val)
    {
        percent = (float)(g_dry_val - raw_soil) * 100.0 / (float)(g_dry_val - g_wet_val);
    }
    
    if (percent < 0) return 0.0;
    if (percent > 100) return 100.0;
    
    return percent;
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

void control_pump(bool is_too_dry, float current_moisture)
{
    if (!g_pump_enabled)
    {
        gpio_set_level(PUMP_RELAY_GPIO, RELAY_OFF);
        return;
    }

    // 1. Wurde gegossen?
    if (g_is_soaking) {
        if (current_moisture < (g_moisture_at_start + 0.5f)) {
            g_retry_count++;
            ESP_LOGW(TAG, "Keine Feuchtigkeitsänderung nach Schlaf (%d/2)", g_retry_count);
            
            if (g_retry_count >= 2) {
                g_dry_run_alarm = true;
                g_pump_enabled = false; // Pumpe komplett sperren
                ESP_LOGE(TAG, "!!! TROCKENLAUF-ALARM !!!");

                if (mqtt_initialized) {
                    // Update den Status für das Dashboard
                    esp_mqtt_client_publish(mqtt_client, "home/plants/basilikum/state_pump_enabled", "0", 0, 1, 1);
                    
                    // NEU: Überschreibe den Retain-Befehl auf dem Broker!
                    // Nimmt HA den Befehl weg, die Automatik beim nächsten Start wieder einzuschalten.
                    esp_mqtt_client_publish(mqtt_client, "home/plants/basilikum/set_pump_enabled", "0", 0, 1, 1);
                    
                    esp_mqtt_client_publish(mqtt_client, "home/plants/basilikum", "{\"dry_run\": 1}", 0, 1, 0);
                }
            }
        } else {
            g_retry_count = 0;
            g_dry_run_alarm = false;
            ESP_LOGI(TAG, "Gießen erfolgreich. Zähler zurückgesetzt.");
        }
        g_is_soaking = false; // Sickerpause beendet
    }

    // 2. Gießen, falls nötig (und kein Alarm anliegt)
    if (is_too_dry && !g_dry_run_alarm) {
        ESP_LOGI(TAG, "Gießvorgang gestartet...");
        g_moisture_at_start = current_moisture;
        
        // Pumpe AN
        gpio_set_level(PUMP_RELAY_GPIO, RELAY_TRIGGER);
        vTaskDelay(pdMS_TO_TICKS(PUMP_DURATION_MS));
        
        // Pumpe AUS
        gpio_set_level(PUMP_RELAY_GPIO, RELAY_OFF);
        
        // Merken, dass wir gegossen haben, damit der nächste Wakeup kontrolliert
        g_is_soaking = true; 
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

    // Luftfeuchtigkeit global
    float hum = 0;
    float temp = 0;
    vTaskDelay(pdMS_TO_TICKS(2000)); // 2 Sekunden warten, bis alle Sensoren stabil sind

// ------------------------- LINEARER ABLAUF (Offline-Fähig) -------------------------

    // 1. Warten auf WLAN, Zeit und MQTT-Sync (Maximal 25 Sekunden)
    ESP_LOGI(TAG, "Warte auf WLAN und Zeit-Sync...");
    int wait_timeout = 0;
    
    while ((!mqtt_initialized || !g_lxh_synced) && wait_timeout < 250) {
        manage_sntp_sync(); 
        vTaskDelay(pdMS_TO_TICKS(100));
        wait_timeout++;
    }

    // 2. Sensoren auslesen (Wird IMMER ausgeführt, auch ohne WLAN)
    if (wait_timeout >= 250) {
        ESP_LOGE(TAG, "Timeout! Kein WLAN/Sync. Führe Notfall-Messung offline aus...");
    } else {
        ESP_LOGI(TAG, "System bereit! Führe Messungen durch...");
    }

    dht_read_float_data(DHT_TYPE_AM2301, DHT_GPIO, &hum, &temp);
    
    float linear_wet = get_smoothed_moisture();
    float vpd = calculate_VPD(temp, hum);
    int current_target = get_dynamic_threshold(g_pump_threshold, vpd);
    bool is_too_dry = (linear_wet < current_target);

    // 3. Pumpen-Logik (Prüft Sickerpause und gießt ggf.)
    control_pump(is_too_dry, linear_wet);

    // 4. Licht auslesen & Daten senden (Nur wenn WLAN da ist)
    float lux = 0;
    bh1750_get_data(bh1750_dev, &lux);

    if (wait_timeout < 250) {
        update_light_logic(lux);
        mqtt_publish_data(lux, vpd, linear_wet, g_lxh_total, g_dry_run_alarm);
        mqtt_publish_debug_data(temp, hum, lux, linear_wet, vpd);
    }

    // 5. Debug Dump (IMMER ausgeben, auch offline wichtig zur Fehleranalyse)
    ESP_LOGI(TAG, "================ DEBUG DUMP ================");
    ESP_LOGI(TAG, "Sensoren Rohwerte:");
    ESP_LOGI(TAG, " - ADC Feuchtigkeit : %d (Kalibrierung -> Nass: %ld, Trocken: %ld)", g_last_raw_soil, g_wet_val, g_dry_val);
    ESP_LOGI(TAG, " - Temperatur       : %.1f °C", temp); 
    ESP_LOGI(TAG, " - Luftfeuchtigkeit : %.1f %%", hum);  
    ESP_LOGI(TAG, " - Licht (aktuell)  : %.0f Lux", lux);
    ESP_LOGI(TAG, "Berechnete Logik:");
    ESP_LOGI(TAG, " - Boden-Prozent    : %.1f %%", linear_wet);
    ESP_LOGI(TAG, " - VPD              : %.2f kPa", vpd);
    ESP_LOGI(TAG, " - Lichtsumme (lxH) : %.1f", g_lxh_total);
    ESP_LOGI(TAG, "System Status:");
    ESP_LOGI(TAG, " - Pumpe Automatik  : %s", g_pump_enabled ? "AN" : "AUS");
    ESP_LOGI(TAG, " - Schwellenwert    : %ld %%", g_pump_threshold);
    ESP_LOGI(TAG, " - Trockenlauf-Alarm: %s", g_dry_run_alarm ? "AKTIV" : "OK");
    ESP_LOGI(TAG, " - Retry-Zähler     : %d/2", g_retry_count);
    ESP_LOGI(TAG, " - Sickerpause      : %s", g_is_soaking ? "JA" : "NEIN");
    ESP_LOGI(TAG, "============================================");

    // 6. Dem ESP kurz Zeit geben, das MQTT-Paket auch wirklich zu senden (wenn online)
    if (wait_timeout < 250) {
        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }

    // ------------------------- GUTE NACHT -------------------------
    ESP_LOGI(TAG, "Gehe in den Deep Sleep für 5 Minuten...");
    esp_sleep_enable_timer_wakeup(5ULL * 60ULL * 1000000ULL); 
    esp_wifi_stop(); 
    esp_deep_sleep_start();
}