idf_component_register(SRCS "src/main.cpp"
                            "src/initDevice/initDevice.cpp"
                            "src/imu/imu.cpp"
                       INCLUDE_DIRS "src"
                       REQUIRES esp_wifi
                                esp_event
                                esp_http_server
                                nvs_flash
                                esp_netif
                                lwip
                                esp_timer
                                log)