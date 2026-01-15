idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_wifi esp_event esp_netif mqtt bh1750 ssd1306 esp_driver_i2c esp_adc nvs_flash dht)