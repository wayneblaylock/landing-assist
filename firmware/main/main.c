#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "softap.h"

static const char *TAG = "main";

void app_main(void)
{
    /* Wi-Fi needs NVS for calibration/config storage */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    //starts the Access Point
    softap_start();

    uint32_t tick = 0;
    while (1) {
        ESP_LOGI(TAG, "tick %lu", (unsigned long)tick++);
        vTaskDelay(pdMS_TO_TICKS(1000));//delay main task loop by 1s
    }
}