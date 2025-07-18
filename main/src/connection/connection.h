#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 初始化 ESP32 为 AP 模式
     * @param ssid AP 的名称
     * @param password AP 的密码（至少8位，若为空则为开放模式）
     */
    void wifi_init_softap(const char *ssid, const char *password);

    void wifi_connect_sta(const char *ssid, const char *password);

#ifdef __cplusplus
}
#endif