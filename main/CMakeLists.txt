idf_component_register(SRCS "src/main.cpp"
                            "src/initDevice/initDevice.cpp"
                            "src/imu/imu.cpp"
                            "src/connection/connection.cpp"
                            "src/webPage/webPage.cpp"
                       INCLUDE_DIRS "src"
                       REQUIRES esp_wifi
                                esp_event
                                esp_http_server
                                nvs_flash
                                esp_netif
                                lwip
                                esp_timer
                                spiffs
                                log
                                json
                                )

spiffs_create_partition_image(spiffs spiffs FLASH_IN_PROJECT)