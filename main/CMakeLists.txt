idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    EMBED_TXTFILES "index.html"
    PRIV_REQUIRES
        esp_wifi
        esp_http_server
        nvs_flash
        esp_netif
        esp_event
        esp_driver_rmt
        esp_driver_gpio
        log
)
