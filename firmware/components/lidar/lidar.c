#include "lidar.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "lidar";

#define BUF_SIZE 128

void lidar_init(void)
{
    uart_config_t cfg = {
        .baud_rate = LIDAR_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(LIDAR_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(LIDAR_UART_NUM, LIDAR_TX_PIN, LIDAR_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(LIDAR_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));

    // Give the sensor a moment after power-up; first command is often ignored
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_LOGI(TAG, "LW20/C UART ready on GPIO%d/GPIO%d @ %d baud",
             LIDAR_TX_PIN, LIDAR_RX_PIN, LIDAR_BAUD);
}

bool lidar_read_slant_m(float *meters)
{
    if (!meters) return false;

    // Flush any leftover bytes
    uart_flush(LIDAR_UART_NUM);

    const char *cmd = "?L\r\n";
    uart_write_bytes(LIDAR_UART_NUM, cmd, strlen(cmd));

    uint8_t buf[BUF_SIZE] = {0};
    int len = uart_read_bytes(LIDAR_UART_NUM, buf, sizeof(buf) - 1, pdMS_TO_TICKS(200));
    if (len <= 0) {
        ESP_LOGW(TAG, "no response");
        return false;
    }
    buf[len] = 0;

    // Expected forms: "l:3.35\r\n" or similar with a colon and a float
    char *colon = strchr((char *)buf, ':');
    if (!colon) {
        ESP_LOGW(TAG, "bad reply: %s", (char *)buf);
        return false;
    }
    float val = strtof(colon + 1, NULL);
    *meters = val;
    return true;
}
