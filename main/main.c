#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h" // Für die bunten Log-Ausgaben

static const char *TAG = "MEIN_PROJEKT";

void app_main(void)
{
    int sekunden = 0;

    while (1) {
        // ESP_LOGI ist schöner als printf (mit Zeitstempel und Farbe)
        ESP_LOGI(TAG, "Hallo Welt! Ich laufe seit %d Sekunden", sekunden++);
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}