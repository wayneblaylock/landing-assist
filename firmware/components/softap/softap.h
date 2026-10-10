#pragma once

#define SOFTAP_SSID      "esp32s3-ap"
#define SOFTAP_PASSWORD  "changeme123"   /* WPA2 requires >= 8 chars */
#define SOFTAP_CHANNEL   6
#define SOFTAP_MAX_CONN  4
#define WIFI_BW          WIFI_BW20 /*avoiding bandwith 40 for power*/
#define BEACON_INTERVAL  300 /*default is 100. Choosing longer to save pwr*/
//map {power, dBm} = {{8, 2}, {20, 5}, {28, 7}, {34, 8}, {44, 11}, {52, 13}, {56, 14}, {60, 15}, {66, 16}, {72, 18}, {80, 20}}
#define TX_PWR           20 /*Range for power is [8, 84] corresponding to 2dBm - 20dBm. 44 -> 11dBm*/

/* Brings up netif, event loop, and the Wi-Fi SoftAP. Call once, after NVS init. */
void softap_start(void);
