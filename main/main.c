//TODO: Error handling (something visual like a blinking led if something fails)

#include <stdint.h>
#include <stddef.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_wifi.h"
#include "esp_http_client.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "lwip/dns.h"
#include "mqtt_client.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "cJSON.h"

#if (LOG_LEVEL_NONE)
#define LOG_LOCAL_LEVEL ESP_LOG_NONE
#elif (LOG_LEVEL_ERROR)
#define LOG_LOCAL_LEVEL ESP_LOG_ERROR
#elif (LOG_LEVEL_WARN)
#define LOG_LOCAL_LEVEL ESP_LOG_WARN
#elif (LOG_LEVEL_INFO)
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#elif (LOG_LEVEL_DEBUG)
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#elif (LOG_LEVEL_VERBOSE)
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#endif
#include "esp_log.h"

static char http_publish_url[256];
static char mqtt_broker_url[256];

static cJSON *root;
static cJSON *devices;

static const char *TAG = "PRESENCE_DETECTOR";

static EventGroupHandle_t wifi_event_group;
const static int CONNECTED_BIT = BIT0;

esp_mqtt_client_handle_t mqtt_client;
static bool mqtt_connected = false;

static void wifi_init(void);
static esp_err_t wifi_event_handler(void *ctx, system_event_t *event);
static void register_on_server(void);
static void bt_init(void);
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void start_ble_scan(void);
static void mqtt_init(void);
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event);
static void add_or_update_device(char *mac, char *name, int rssi);
static void build_and_publish_device_list(void);
static void publish_devices_via_http(char *payload);
static esp_err_t _http_event_handler(esp_http_client_event_t *evt);
static void publish_device_via_mqtt(char *payload);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

struct gattc_profile_inst
{
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};

void app_main()
{
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    wifi_init();
}

static void wifi_init(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_LOGI(TAG, "start the WIFI SSID:[%s]", CONFIG_WIFI_SSID);
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Waiting for wifi...");
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
}

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id)
    {
    case SYSTEM_EVENT_STA_START:
        ESP_LOGD(TAG, "Connecting wifi...");
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        ESP_LOGD(TAG, "Wifi connected successfull.");
        register_on_server();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        ESP_LOGD(TAG, "Wifi connection lost! Try to reconnect...");
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        esp_wifi_connect();
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void register_on_server(void)
{
#if (CONFIG_GET_URLS_FROM_BROADCAST)
    ESP_LOGI(TAG, "Sending register request...");

    char *message = "Register request from BLE Presence Detector";
    char rx_buffer[128];
    char addr_str[128];

    struct sockaddr_in destAddr;
    destAddr.sin_addr.s_addr = inet_addr("255.255.255.255");
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(CONFIG_REGISTER_BROADCAST_PORT);
    inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return;
    }
    ESP_LOGD(TAG, "Socket created");

    int err = sendto(sock, message, strlen(message), 0, (struct sockaddr *)&destAddr, sizeof(destAddr));
    if (err < 0)
    {
        ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
        return;
    }
    ESP_LOGD(TAG, "Register request sent");

    struct sockaddr_in sourceAddr;
    socklen_t socklen = sizeof(sourceAddr);
    int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&sourceAddr, &socklen);

    if (len < 0)
    {
        ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
        return;
    }
    else
    {
        rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
        ESP_LOGD(TAG, "Received %d bytes from %s:", len, addr_str);

        ESP_LOGD(TAG, "Got register response. Parsing...");

        cJSON *root = cJSON_Parse(rx_buffer);

        if (root == NULL)
        {
            const char *error_ptr = cJSON_GetErrorPtr();
            if (error_ptr != NULL)
            {
                fprintf(stderr, "Error parsing response before: %s\n", error_ptr);
            }
            return;
        }

        ESP_LOGD(TAG, "Parsing was successfull.");

        cJSON *http_url = cJSON_GetObjectItemCaseSensitive(root, "http");
        if (cJSON_IsString(http_url) && (http_url->valuestring != NULL))
        {
            strcpy(http_publish_url, http_url->valuestring);
            ESP_LOGD(TAG, "http publish url set to: %s", http_publish_url);
        }

        cJSON *mqtt_url = cJSON_GetObjectItemCaseSensitive(root, "mqtt");
        if (cJSON_IsString(mqtt_url) && (mqtt_url->valuestring != NULL))
        {
            strcpy(mqtt_broker_url, mqtt_url->valuestring);
            ESP_LOGD(TAG, "mqtt broker url set to: %s", mqtt_broker_url);
        }

        cJSON_Delete(root);

        ESP_LOGI(TAG, "Registering done. Starting...");

        bt_init();
        mqtt_init();
    }

    if (sock != -1)
    {
        ESP_LOGD(TAG, "Shutting down broadcast socket...");
        shutdown(sock, 0);
        close(sock);
    }

#else
    ESP_LOGI(TAG, "Skipping register request...");
    static char *http_publish_url = CONFIG_HTTP_POST_PUBLISH_URL;
    static char *mqtt_broker_url = CONFIG_MQTT_BROKER_URL;
    bt_init();
    mqtt_init();
#endif
}

static void bt_init(void)
{
    ESP_LOGI(TAG, "Starting Bluetooth...");
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret)
    {
        ESP_LOGE(TAG, "%s gap register failed, error code = %x\n", __func__, ret);
        return;
    }

    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if (ret)
    {
        ESP_LOGE(TAG, "%s gattc register failed, error code = %x\n", __func__, ret);
        return;
    }

    ret = esp_ble_gattc_app_register(0);
    if (ret)
    {
        ESP_LOGE(TAG, "%s gattc app register failed, error code = %x\n", __func__, ret);
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret)
    {
        ESP_LOGE(TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    switch (event)
    {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
    {
        start_ble_scan();
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(TAG, "scan start failed, error status = %x", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGD(TAG, "scan start success\n");
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
    {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt)
        {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            ESP_LOGD(TAG, "BLE device found:");
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, scan_result->scan_rst.bda, 6, ESP_LOG_DEBUG);

            uint8_t *bt_address = scan_result->scan_rst.bda;
            char mac[18];
            sprintf(mac, "%02x:%02x:%02x:%02x:%02x:%02x", bt_address[0], bt_address[1], bt_address[2], bt_address[3], bt_address[4], bt_address[5]);

            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            ESP_LOG_BUFFER_CHAR_LEVEL(TAG, adv_name, adv_name_len, ESP_LOG_DEBUG);

            int rssi = scan_result->scan_rst.rssi;

            add_or_update_device(mac, (char *)adv_name, rssi);
            break;

        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            ESP_LOGD(TAG, "BLE scan done.\n");
            ESP_LOGD(TAG, "Sending devices...");
            build_and_publish_device_list();
            ESP_LOGD(TAG, "suspending %d seconds...", CONFIG_BLE_SCAN_INTERVAL);
            sleep(CONFIG_BLE_SCAN_INTERVAL);
            start_ble_scan();
            break;
        default:
            break;
        }
        break;
    }
    default:
        break;
    }
}

static void start_ble_scan(void)
{
    root = cJSON_CreateObject();
    devices = cJSON_CreateArray();
    cJSON_AddItemToObject(root, "devices", devices);

    ESP_LOGD(TAG, "Starting BLE scan...\n");
    esp_ble_gap_start_scanning((uint32_t)CONFIG_BLE_SCAN_DURATION);
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    struct gattc_profile_inst gl_profile_tab[1] = {
        [0] = {
            .gattc_cb = gattc_profile_event_handler,
            .gattc_if = ESP_GATT_IF_NONE,
        },
    };
    if (event == ESP_GATTC_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        }
        else
        {
            ESP_LOGI(TAG, "reg app failed, app_id %04x, status %d",
                     param->reg.app_id,
                     param->reg.status);
            return;
        }
    }

    do
    {
        int idx;
        for (idx = 0; idx < 1; idx++)
        {
            if (gattc_if == ESP_GATT_IF_NONE || 
                gattc_if == gl_profile_tab[idx].gattc_if)
            {
                if (gl_profile_tab[idx].gattc_cb)
                {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    if (event == ESP_GATTC_REG_EVT)
    {
        ESP_LOGD(TAG, "Setting bluetooth params...");
        esp_ble_scan_params_t ble_scan_params = {
            .scan_type = BLE_SCAN_TYPE_ACTIVE,
            .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
            .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
            .scan_interval = 0x50,
            .scan_window = 0x30,
            .scan_duplicate = BLE_SCAN_DUPLICATE_ENABLE};
        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret)
        {
            ESP_LOGE(TAG, "set scan params error, error code = %x", scan_ret);
        }
    }
}

static void mqtt_init(void)
{
#if (CONFIG_PUBLISH_PROTOCOL_MQTT || CONFIG_PUBLISH_PROTOCOL_ALL)
    ESP_LOGI(TAG, "Connecting to mqtt broker...");
    ESP_LOGD(TAG, "mqtt broker url: %s", mqtt_broker_url);
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = mqtt_broker_url};

    mqtt_client = (esp_mqtt_client_handle_t)esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, mqtt_client);
    esp_mqtt_client_start(mqtt_client);
#endif
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    switch (event->event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGD(TAG, "MQTT_EVENT_CONNECTED");
        mqtt_connected = true;
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "MQTT_EVENT_DISCONNECTED");
        mqtt_connected = false;
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGD(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGD(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGD(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGD(TAG, "MQTT_EVENT_DATA");
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT_EVENT_ERROR");
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void add_or_update_device(char *mac, char *name, int rssi)
{
    int index = 0;
    bool foundDeviceToUpdate = false;
    cJSON *iterationDevice;
    cJSON_ArrayForEach(iterationDevice, devices)
    {
        char *macAddress = cJSON_GetObjectItemCaseSensitive(iterationDevice, "mac")->valuestring;
        if (strcmp(macAddress, mac) == 0)
        {
            foundDeviceToUpdate = true;
            break;
        }
        index++;
    }

    if (foundDeviceToUpdate)
    {
        ESP_LOGD(TAG, "Updating device in device list\n");
        cJSON_DeleteItemFromArray(devices, index);
    }
    else
    {
        ESP_LOGD(TAG, "Adding device device to device tree...\n");
    }

    cJSON *device = cJSON_CreateObject();
    cJSON_AddItemToArray(devices, device);

    cJSON *macAddress = cJSON_CreateString(mac);
    cJSON_AddItemToObject(device, "mac", macAddress);

    cJSON *rssiValue = cJSON_CreateNumber(rssi);
    cJSON_AddItemToObject(device, "rssi", rssiValue);

    if (name != NULL)
    {
        cJSON *deviceName = cJSON_CreateString(name);
        cJSON_AddItemToObject(device, "name", deviceName);
    }
}

static void build_and_publish_device_list(void)
{
    ESP_LOGD(TAG, "Building json...");

    cJSON *from = cJSON_CreateString(CONFIG_PRESENCE_DETECTOR_NAME);
    cJSON_AddItemToObject(root, "from", from);

    char *json = cJSON_Print(root);

    publish_devices_via_http(json);
    publish_device_via_mqtt(json);
}

static void publish_devices_via_http(char *payload)
{
#if (CONFIG_PUBLISH_PROTOCOL_HTTP_POST || CONFIG_PUBLISH_PROTOCOL_ALL)
    ESP_LOGD(TAG, "Sending http post request...");
    ESP_LOGD(TAG, "Using url: %s", http_publish_url);

    esp_http_client_config_t config = {
        .url = http_publish_url,
        .event_handler = _http_event_handler};
    esp_http_client_handle_t client = esp_http_client_init(&config);

    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_post_field(client, payload, strlen(payload));
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_err_t err = err = esp_http_client_perform(client);
    if (err == ESP_OK)
    {
        ESP_LOGD(TAG, "HTTP POST Status = %d, content_length = %d",
                 esp_http_client_get_status_code(client),
                 esp_http_client_get_content_length(client));
    }
    else
    {
        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
#endif
}

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    }
    return ESP_OK;
}

static void publish_device_via_mqtt(char *payload)
{
#if (CONFIG_PUBLISH_PROTOCOL_MQTT || CONFIG_PUBLISH_PROTOCOL_ALL)
    ESP_LOGD(TAG, "Sending mqtt request...");
    if (mqtt_connected)
    {
        char topic[256];
        strcpy(topic, "/ble-presence-detector/");
        strcat(topic, CONFIG_PRESENCE_DETECTOR_NAME);

        esp_mqtt_client_publish(mqtt_client, topic, payload, strlen(payload), 0, 0);
    }
#endif
}