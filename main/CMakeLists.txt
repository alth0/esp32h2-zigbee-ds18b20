idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES esp-zboss-lib esp-zigbee-lib ds18b20 onewire_bus nvs_flash esp_event
)