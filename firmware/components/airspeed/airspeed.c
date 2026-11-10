#include "airspeed.h"

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "airspeed";

// Type A constants, ±1 PSI range
#define P_CNT     16383.0f
#define C_A       0.1f
#define D_A       0.8f
#define P_MAX     1.0f
#define P_MIN    -1.0f
#define PSI_TO_PA 6894.757f

void airspeed_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = AIRSPEED_SDA_PIN,
        .scl_io_num = AIRSPEED_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = AIRSPEED_I2C_FREQ,
    };
    ESP_ERROR_CHECK(i2c_param_config(AIRSPEED_I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(AIRSPEED_I2C_PORT, conf.mode, 0, 0, 0));

    // Quick probe of the expected address
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AIRSPEED_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(AIRSPEED_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "MS4525DO ACK at 0x%02X", AIRSPEED_I2C_ADDR);
    } else {
        ESP_LOGW(TAG, "no ACK at 0x%02X (%s) — check 5 V on sensor pin 2",
                 AIRSPEED_I2C_ADDR, esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "I2C ready (SDA=%d SCL=%d)", AIRSPEED_SDA_PIN, AIRSPEED_SCL_PIN);
}

bool airspeed_read_diff_pa(float *pressure_pa, float *temp_c)
{
    // Measurement request
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AIRSPEED_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(AIRSPEED_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "MR failed: %s", esp_err_to_name(err));
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(3));

    // Data fetch
    uint8_t buf[4] = {0};
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AIRSPEED_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buf, 3, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &buf[3], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(AIRSPEED_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "read failed: %s", esp_err_to_name(err));
        return false;
    }

    uint8_t status = (buf[0] >> 6) & 0x03;
    if (status == 3) {
        ESP_LOGW(TAG, "sensor fault status");
        return false;
    }

    uint16_t p_cnts = ((buf[0] & 0x3F) << 8) | buf[1];
    uint16_t t_cnts = (buf[2] << 3) | (buf[3] >> 5);

    float psi = (p_cnts - C_A * P_CNT) * ((P_MAX - P_MIN) / (D_A * P_CNT)) + P_MIN;
    float pa = psi * PSI_TO_PA;
    float temp = (t_cnts / 2047.0f) * 200.0f - 50.0f;

    if (pressure_pa) *pressure_pa = pa;
    if (temp_c) *temp_c = temp;
    return true;
}
