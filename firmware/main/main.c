#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "softap.h"
#include "webserver.h"
#include "attitude.h"

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

    // Start the SoftAP
    softap_start();

    //start the ARS (attitude reference system)
    attitude_start();

    // Start the HTTP server that serves the ADI
    webserver_start();

    

    uint32_t tick = 0;
    while (1) {
        ESP_LOGI(TAG, "tick %lu", (unsigned long)tick++);

        attitude_t att;
        if (attitude_get(&att, pdMS_TO_TICKS(20))) {
            ESP_LOGI(TAG, "PITCH/ROLL:  %.3f %.3f", att.pitch, att.roll);
        } else {
            ESP_LOGW(TAG, "no attitude data");
        }

        if (tick == 10) {attitude_request_calibration(pdMS_TO_TICKS(5000));}

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
