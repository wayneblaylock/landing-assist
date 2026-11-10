#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "lidar.h"
#include "airspeed.h"

static const char *TAG = "main";

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_LOGI(TAG, "Sensor bring-up (LiDAR + airspeed)");
    lidar_init();
    airspeed_init();

    while (1) {
        float meters = 0.0f;
        if (lidar_read_slant_m(&meters)) {
            ESP_LOGI(TAG, "slant range: %.2f m", meters);
        } else {
            ESP_LOGW(TAG, "lidar read failed");
        }

        float pa = 0.0f, temp = 0.0f;
        if (airspeed_read_diff_pa(&pa, &temp)) {
            ESP_LOGI(TAG, "diff pressure: %.1f Pa  temp: %.1f C", pa, temp);
        } else {
            ESP_LOGW(TAG, "airspeed read failed");
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
