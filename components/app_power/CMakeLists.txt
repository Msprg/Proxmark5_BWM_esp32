# Register component
idf_component_register(SRCS "app_power.c" INCLUDE_DIRS "." REQUIRES app_ble_spp PRIV_REQUIRES esp_pm)
