#include "airspeed.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "airspeed";

static i2c_master_bus_handle_t bus = NULL;
static i2c_master_dev_handle_t dev = NULL;

// Type A constants, ±1 PSI range
#define P_CNT     16383.0f
#define C_A       0.1f
#define D_A       0.8f
#define P_MAX     1.0f
#define P_MIN    -1.0f
#define PSI_TO_PA 6894.757f

void airspeed_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = AIRSPEED_I2C_PORT,
        .sda_io_num = AIRSPEED_SDA_PIN,
        .scl_io_num = AIRSPEED_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AIRSPEED_I2C_ADDR,
        .scl_speed_hz = AIRSPEED_I2C_FREQ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &dev));

    ESP_LOGI(TAG, "MS4525DO ready on I2C (SDA=%d SCL=%d addr=0x%02X)",
             AIRSPEED_SDA_PIN, AIRSPEED_SCL_PIN, AIRSPEED_I2C_ADDR);
}

bool airspeed_read_diff_pa(float *pressure_pa, float *temp_c)
{
    if (!dev) return false;

    uint8_t buf[4] = {0};
    esp_err_t err = i2c_master_receive(dev, buf, sizeof(buf), pdMS_TO_TICKS(50));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "I2C read failed: %s", esp_err_to_name(err));
        return false;
    }

    uint8_t status = (buf[0] >> 6) & 0x03;
    if (status == 2 || status == 3) {
        ESP_LOGW(TAG, "sensor status fault (%u)", status);
        return false;
    }

    uint16_t p_cnts = ((buf[0] & 0x3F) << 8) | buf[1];
    uint16_t t_cnts = (buf[2] << 3) | (buf[3] >> 5);

    // Type A, ±1 PSI
    float psi = (p_cnts - C_A * P_CNT) * ((P_MAX - P_MIN) / (D_A * P_CNT)) + P_MIN;
    float pa = psi * PSI_TO_PA;

    // Temperature: 11-bit, roughly -50 to 150 °C
    float temp = (t_cnts / 2047.0f) * 200.0f - 50.0f;

    if (pressure_pa) *pressure_pa = pa;
    if (temp_c) *temp_c = temp;
    return true;
}
