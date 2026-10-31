#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "softap.h"
#include "webserver.h"
#include "bmi088.h"

static const char *TAG = "main";

void vTestBMI_FIFO(void* pvParameters) {

    bmi088_config_t bmi088_cfg = BMI088_DEFAULT_CONFIG();

    bmi088_cfg.spi_clock_speed_hz = 8 * 1000 * 1000;
    bmi088_cfg.acc_fifo_en = 0x01;
    bmi088_cfg.acc_odr = ACC_ODR_200_HZ;
    bmi088_cfg.gyro_bw = GYRO_BW_64;//200HZ ODR

    bmi088_init(&bmi088_cfg);
    float accel[450];
    float gyro[450];
    int i = 0;

    while (1) {
        int accel_values_read = acc_read_fifo(accel, 450);
        int gyro_values_read = gyro_read_fifo(gyro, 450);
        //only print every 30 values
        if ((i % 60) == 0) {
            //ESP_LOGI(TAG, "BMI088Accel read in %d values from FIFO", values_read);
            //ESP_LOGI(TAG, "BMI088Accel %.6f %.6f %.6f", accel[0],accel[1],accel[2]);
        }
        //ESP_LOGI(TAG, "BMI088Accel data rate register is 0x%X values from FIFO. Expect 0x%X", acc_read_reg(0x40), (bmi088_cfg.acc_bwp << 4) | (bmi088_cfg.acc_odr & 0x0F));
        ESP_LOGI(TAG, "BMI088Gyroroscope   read in %d values from FIFO. First 3: %.6f %.6f %.6f", accel_values_read, accel[accel_values_read-3],accel[accel_values_read-2],accel[accel_values_read-1]);
        ESP_LOGI(TAG, "BMI088Accelerometer read in %d values from FIFO. First 3: %.6f %.6f %.6f", gyro_values_read, gyro[gyro_values_read-3],gyro[gyro_values_read-2],gyro[gyro_values_read-1]);


        vTaskDelay(pdMS_TO_TICKS(1600));//60Hz???
        i++;
    }
}

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

    // Start the HTTP server that serves the ADI
    webserver_start();

    xTaskCreate(vTestBMI_FIFO, "IMU", 6096, NULL, 1, NULL);

    uint32_t tick = 0;
    while (1) {
        ESP_LOGI(TAG, "tick %lu", (unsigned long)tick++);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
