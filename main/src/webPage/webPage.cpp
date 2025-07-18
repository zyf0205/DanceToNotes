#include "webPage.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include <stdio.h>
#include <string>
#include <string.h>
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "connection/connection.h"

static const char *TAG = "WEB_PAGE";
static httpd_handle_t server = NULL;

// 传感器数据存储
static float current_accel_x = 0.0f;
static float current_accel_y = 0.0f;
static float current_accel_z = 0.0f;
static float current_gyro_x = 0.0f;
static float current_gyro_y = 0.0f;
static float current_gyro_z = 0.0f;

static std::string read_html_file(const char *path)
{
    FILE *file = fopen(path, "r");
    if (!file)
        return "<html><body><h1>无法加载网页</h1></body></html>";
    std::string content;
    char buf[256];
    while (fgets(buf, sizeof(buf), file))
    {
        content += buf;
    }
    fclose(file);
    return content;
}

// WiFi配置POST处理
esp_err_t wifi_config_post_handler(httpd_req_t *req)
{
    char buf[256];
    int ret = httpd_req_recv(req, buf, sizeof(buf));
    if (ret <= 0)
    {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root)
    {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    cJSON *ssid = cJSON_GetObjectItem(root, "ssid");
    cJSON *password = cJSON_GetObjectItem(root, "password");

    if (ssid && password && cJSON_IsString(ssid) && cJSON_IsString(password))
    {
        // 保存配置到NVS
        // save_wifi_config(ssid->valuestring, password->valuestring);

        printf("ssid:%s  password:%s\r\n", ssid->valuestring, password->valuestring);

        // 切换到STA模式并连接
        wifi_connect_sta(ssid->valuestring, password->valuestring);

        // 返回响应，不重启
        const char *response = "{\"success\":true,\"message\":\"WiFi配置成功，设备正在尝试连接...\"}";
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, response, strlen(response));
    }
    else
    {
        httpd_resp_send_500(req);
    }
    cJSON_Delete(root);
    return ESP_OK;
}

// 传感器数据GET处理
esp_err_t sensor_data_get_handler(httpd_req_t *req)
{
    // 构造JSON响应
    cJSON *root = cJSON_CreateObject();
    cJSON *accel = cJSON_CreateObject();
    cJSON *gyro = cJSON_CreateObject();

    cJSON_AddItemToObject(root, "accel", accel);
    cJSON_AddItemToObject(root, "gyro", gyro);

    cJSON_AddNumberToObject(accel, "x", current_accel_x);
    cJSON_AddNumberToObject(accel, "y", current_accel_y);
    cJSON_AddNumberToObject(accel, "z", current_accel_z);

    cJSON_AddNumberToObject(gyro, "x", current_gyro_x);
    cJSON_AddNumberToObject(gyro, "y", current_gyro_y);
    cJSON_AddNumberToObject(gyro, "z", current_gyro_z);

    char *json_string = cJSON_Print(root);
    cJSON_Delete(root);

    if (json_string)
    {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, json_string, strlen(json_string));
        free(json_string);
    }
    else
    {
        httpd_resp_send_500(req);
    }

    return ESP_OK;
}

// 设备控制POST处理
esp_err_t device_control_post_handler(httpd_req_t *req)
{
    char buf[512];
    int ret = httpd_req_recv(req, buf, sizeof(buf));
    if (ret <= 0)
    {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    buf[ret] = '\0';
    ESP_LOGI(TAG, "收到设备控制: %s", buf);

    cJSON *root = cJSON_Parse(buf);
    if (root)
    {
        cJSON *action = cJSON_GetObjectItem(root, "action");

        if (action && cJSON_IsString(action))
        {
            if (strcmp(action->valuestring, "restart") == 0)
            {
                ESP_LOGI(TAG, "设备重启请求");
                const char *response = "{\"success\":true,\"message\":\"设备正在重启...\"}";
                httpd_resp_set_type(req, "application/json");
                httpd_resp_send(req, response, strlen(response));

                // 延迟重启
                vTaskDelay(pdMS_TO_TICKS(1000));
                esp_restart();
            }
            else if (strcmp(action->valuestring, "reset_wifi") == 0)
            {
                ESP_LOGI(TAG, "重置WiFi配置请求");
                // TODO: 清除WiFi配置
                const char *response = "{\"success\":true,\"message\":\"WiFi配置已重置\"}";
                httpd_resp_set_type(req, "application/json");
                httpd_resp_send(req, response, strlen(response));
            }
            else
            {
                const char *response = "{\"success\":false,\"message\":\"未知的控制命令\"}";
                httpd_resp_set_type(req, "application/json");
                httpd_resp_send(req, response, strlen(response));
            }
        }
        cJSON_Delete(root);
    }
    else
    {
        const char *response = "{\"success\":false,\"message\":\"JSON解析失败\"}";
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, response, strlen(response));
    }

    return ESP_OK;
}

esp_err_t link_get_handler(httpd_req_t *req)
{
    std::string html = read_html_file("/spiffs/link.html");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html.c_str(), html.length());
    return ESP_OK;
}

esp_err_t function_get_handler(httpd_req_t *req)
{
    std::string html = read_html_file("/spiffs/function.html");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html.c_str(), html.length());
    return ESP_OK;
}

void start_web_server()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = 4;

    if (httpd_start(&server, &config) == ESP_OK)
    {
        // 注册主页（可根据实际需求选择页面）
        httpd_uri_t home = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = link_get_handler, // 或 link_get_handler，或根据mode动态选择
            .user_ctx = NULL};
        httpd_register_uri_handler(server, &home);

        // 注册WiFi配网接口
        httpd_uri_t wifi_config = {
            .uri = "/wifi-config",
            .method = HTTP_POST,
            .handler = wifi_config_post_handler,
            .user_ctx = NULL};
        httpd_register_uri_handler(server, &wifi_config);

        // 注册传感器数据接口
        httpd_uri_t sensor_data = {
            .uri = "/sensor-data",
            .method = HTTP_GET,
            .handler = sensor_data_get_handler,
            .user_ctx = NULL};
        httpd_register_uri_handler(server, &sensor_data);

        // 注册设备控制接口
        httpd_uri_t device_control = {
            .uri = "/device-control",
            .method = HTTP_POST,
            .handler = device_control_post_handler,
            .user_ctx = NULL};
        httpd_register_uri_handler(server, &device_control);

        ESP_LOGI(TAG, "Web server started");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to start web server");
    }
}

// 更新传感器数据
void update_sensor_data(float accel_x, float accel_y, float accel_z,
                        float gyro_x, float gyro_y, float gyro_z)
{
    current_accel_x = accel_x;
    current_accel_y = accel_y;
    current_accel_z = accel_z;
    current_gyro_x = gyro_x;
    current_gyro_y = gyro_y;
    current_gyro_z = gyro_z;
}

// 停止web服务器
void stop_web_server(void)
{
    if (server)
    {
        httpd_stop(server);
        server = NULL;
        ESP_LOGI(TAG, "Web server stopped");
    }
}