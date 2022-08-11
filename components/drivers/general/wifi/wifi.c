/**
 * @brief 文件描述：待更新
 * @author 小鱼 (fishros@foxmail.com)
 * @version V1.0.0
 * @date 2022-07-07
 * @copyright 版权所有：FishBot Open Source Organization
 */
#include "wifi.h"

#define FISHBOT_MODLUE "WIFI"
#define DEFAULT_WIFI_AP_PSWD "fishros.com"
#define DEFAULT_WIFI_AP_SSID_PREFIX "FISHBOT"

static fishbot_wifi_config_t *wifi_config_ = NULL;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

static wifi_status_t wifi_status_ = WIFI_STATUS_STA_DISCONECTED;
static char wifi_ip_[16];
static uint8_t retry_connect_sta_count_ = 0;

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        // if (s_retry_num < 3) {
        wifi_status_ = WIFI_STATUS_STA_DISCONECTED;
        retry_connect_sta_count_++;
        esp_wifi_connect();
        ESP_LOGI(FISHBOT_MODLUE, "retry to connect to the AP");
        // } else {
        // xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        // }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(FISHBOT_MODLUE, "got ip:%s", ip4addr_ntoa(&event->ip_info.ip));
        sprintf(wifi_ip_, "%s", ip4addr_ntoa(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        wifi_status_ = WIFI_STATUS_STA_CONNECTED;
    }
    else if (event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t *event =
            (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(FISHBOT_MODLUE, "station " MACSTR " join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *event =
            (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(FISHBOT_MODLUE, "station " MACSTR " leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

bool set_wifi_config(fishbot_wifi_config_t *wifi_config)
{
    wifi_config_ = wifi_config;
    return true;
}

/**
 * @brief 发射一个热点
 *
 * @param ssid
 * @param pswd
 * @return true
 * @return false
 */
bool wifi_set_as_ap(char *ssid, char *pswd)
{
    uint8_t mac[6];
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .ap =
            {
                .max_connection = WIF_MAX_AP_CONNECTION,
                .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            },
    };

    ESP_ERROR_CHECK(esp_wifi_get_mac(ESP_IF_WIFI_AP, mac));
    wifi_config.ap.ssid_len =
        sprintf((char *)wifi_config.ap.ssid, "%s_%02X%02X", ssid, mac[0], mac[1]);
    // sprintf(ssid, "%s_%02X%02X%02X%02X%02X%02X", ssid, mac[0], mac[1], mac[2],
    // mac[3], mac[4], mac[5]); strcpy((char *)wifi_config.ap.ssid, ssid);
    strcpy((char *)wifi_config.ap.password, pswd);

    if (strlen(pswd) == 0)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_LOGI(FISHBOT_MODLUE, "wifi_init_softap finished.SSID:%s password:%s",
             ssid, pswd);
    wifi_status_ = WIFI_STATUS_AP_READY;
    return true;
}

bool wifi_set_as_sta(char *ssid, char *password)
{
    wifi_config_t wifi_config = {
        .sta =
            {
                .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            },
    };
    strcpy((char *)wifi_config.ap.ssid, ssid);
    strcpy((char *)wifi_config.ap.password, password);

    s_wifi_event_group = xEventGroupCreate();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(FISHBOT_MODLUE, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or
     * connection failed for the maximum number of re-tries (WIFI_FAIL_BIT). The
     * bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE,
        10 * 1000 / portTICK_RATE_MS);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we
     * can test which event actually happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(FISHBOT_MODLUE, "connected to ap SSID:%s password:%s",
                 wifi_config.ap.ssid, wifi_config.ap.password);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(FISHBOT_MODLUE, "Failed to connect to SSID:%s, password:%s",
                 wifi_config.ap.ssid, wifi_config.ap.password);
    }
    else
    {
        ESP_LOGE(FISHBOT_MODLUE, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(
        IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(
        WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
    if (wifi_status_ != WIFI_STATUS_STA_CONNECTED)
    {
        return true;
    }
    return false;
}

bool wifi_init()
{

    if (wifi_config_ == NULL)
    {
        ESP_LOGE(FISHBOT_MODLUE, "wifi_config_ is not config!");
        return false;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    // bool result;
    if (wifi_config_->mode == WIFI_MODE_STA)
    {
        ESP_LOGI(FISHBOT_MODLUE, "wifi init with sta mode, ssid:%s pswd:%s",
                 wifi_config_->ssid, wifi_config_->password);
        wifi_set_as_sta(wifi_config_->ssid, wifi_config_->password);
    }
    if (wifi_config_->mode == WIFI_MODE_AP)
    {
        ESP_LOGI(FISHBOT_MODLUE, "wifi init with ap mode, ssid:%s pswd:%s",
                DEFAULT_WIFI_AP_SSID_PREFIX, DEFAULT_WIFI_AP_PSWD);
        wifi_set_as_ap(DEFAULT_WIFI_AP_SSID_PREFIX, DEFAULT_WIFI_AP_PSWD);
    }
    // 初始化失败情况下自动选择ap模式
    if (wifi_status_ == WIFI_STATUS_STA_DISCONECTED)
    {
        ESP_LOGI(FISHBOT_MODLUE,
                 "wifi init falied! try with ap mode, ssid:%s pswd:%s",
                 DEFAULT_WIFI_AP_SSID_PREFIX, DEFAULT_WIFI_AP_PSWD);
        wifi_set_as_ap(DEFAULT_WIFI_AP_SSID_PREFIX, DEFAULT_WIFI_AP_PSWD);
    }

    ESP_LOGI(FISHBOT_MODLUE, "init success!");
    return true;
}

wifi_status_t get_wifi_ip(char *ip_address)
{
    if (wifi_status_ == WIFI_STATUS_AP_READY)
    {
        sprintf(ip_address, "192.168.4.1");
    }
    else if (wifi_status_ == WIFI_STATUS_STA_CONNECTED)
    {
        sprintf(ip_address, "%s", wifi_ip_);
    }
    else if (wifi_status_ == WIFI_STATUS_STA_DISCONECTED)
    {
        sprintf(ip_address, "lost_connect");
    }
    return wifi_status_;
}