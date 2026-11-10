#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "lidar.h"

static const char *TAG = "main";

void app_main(void)
{
    // NVS still needed for some subsystems; keep the init
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_LOGI(TAG, "LW20/C slant-range bring-up");
    lidar_init();

    while (1) {
        float meters = 0.0f;
        if (lidar_read_slant_m(&meters)) {
            ESP_LOGI(TAG, "slant range: %.2f m", meters);
        } else {
            ESP_LOGW(TAG, "read failed");
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
