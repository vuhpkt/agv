idf_component_register(SRCS "main.c" "ble_cmd_dispatcher.c"
                       INCLUDE_DIRS "."
                       REQUIRES "uart_service" "button" "ble_nus" "comms_protocol" "nvs_flash")
