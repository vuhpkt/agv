idf_component_register(SRCS "ble_nus.c"
                       INCLUDE_DIRS "include"
                       REQUIRES "bt" "nvs_flash"
                       PRIV_REQUIRES "freertos" "log")

