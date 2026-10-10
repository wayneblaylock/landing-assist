#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include "softap.h"

static const char *TAG = "softap";

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)data;
        ESP_LOGI(TAG, "station " MACSTR " joined, AID=%d", MAC2STR(e->mac), e->aid);
    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *e = (wifi_event_ap_stadisconnected_t *)data;
        ESP_LOGI(TAG, "station " MACSTR " left, AID=%d", MAC2STR(e->mac), e->aid);
    }
}

void softap_start(void)
{
    //must be called once
    ESP_ERROR_CHECK(esp_netif_init());
    //using a default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    //quick ap startup. We don't save the handle here.
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &wifi_event_handler, NULL));

    //Wifi configuration settings (ssid, pass, ect)
    wifi_config_t wifi_config = {
        .ap = {
            .ssid           = SOFTAP_SSID,
            .ssid_len       = strlen(SOFTAP_SSID),
            .channel        = SOFTAP_CHANNEL,
            .password       = SOFTAP_PASSWORD,
            .max_connection = SOFTAP_MAX_CONN,
            .authmode       = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg        = { .required = false },
            .beacon_interval = BEACON_INTERVAL,
        },
    };

    //set mode to access point
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    
    //apply config
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    
    // avoiding 802.11b since it consumes more power
    //ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL));

    //set bandwidth
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW));
    
    //start wifi
    ESP_ERROR_CHECK(esp_wifi_start());

    //set transmit power.
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(TX_PWR));

    ESP_LOGI(TAG, "SoftAP up. SSID:%s channel:%d", SOFTAP_SSID, SOFTAP_CHANNEL);
}
