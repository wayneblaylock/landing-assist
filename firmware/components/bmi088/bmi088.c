#include <stdio.h>
#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bmi088.h"

#define TAG "bmi088"

static spi_device_handle_t bmi088_acc;
static spi_device_handle_t bmi088_gyro;


/******************ACCELEROMETER**********************/

//register values
#define ACC_ID                      0x1E
#define ACC_PWR_CONF_ACTIVE         0x00
#define ACC_PWR_CONF_SUSPEND        0x03
#define ACC_PWR_CTRL_ON             0x04
#define ACC_PWR_CTRL_OFF            0x00
#define ACC_SOFTRESET_CMD           0xB6
#define ACC_FIFO_RESET_CMD          0xB0   // written to ACC_SOFTRESET

//register names
#define REG_ACC_CHIP_ID             0x00   // reset: 0x1E
#define REG_ACC_ERR_REG             0x02
#define REG_ACC_STATUS              0x03
#define REG_ACC_DATA_START          0x12   // X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB (0x12-0x17)
#define REG_ACC_X_LSB               0x12
#define REG_ACC_X_MSB               0x13
#define REG_ACC_Y_LSB               0x14
#define REG_ACC_Y_MSB               0x15
#define REG_ACC_Z_LSB               0x16
#define REG_ACC_Z_MSB               0x17
#define REG_ACC_SENSORTIME_0        0x18
#define REG_ACC_SENSORTIME_1        0x19
#define REG_ACC_SENSORTIME_2        0x1A
#define REG_ACC_INT_STAT_1          0x1D
#define REG_ACC_TEMP_MSB            0x22
#define REG_ACC_TEMP_LSB            0x23
#define REG_ACC_FIFO_LENGTH_0       0x24
#define REG_ACC_FIFO_LENGTH_1       0x25
#define REG_ACC_FIFO_DATA           0x26
#define REG_ACC_CONF                0x40
#define REG_ACC_RANGE               0x41
#define REG_ACC_FIFO_DOWNS          0x45
#define REG_ACC_FIFO_WTM_0          0x46
#define REG_ACC_FIFO_WTM_1          0x47
#define REG_ACC_FIFO_CONFIG_0       0x48
#define REG_ACC_FIFO_CONFIG_1       0x49
#define REG_ACC_INT1_IO_CTRL        0x53   // "INT1_IO_CONF" in register description
#define REG_ACC_INT2_IO_CTRL        0x54   // "INT2_IO_CONF" in register description
#define REG_ACC_INT1_INT2_MAP_DATA  0x58
#define REG_ACC_SELF_TEST           0x6D
#define REG_ACC_PWR_CONF            0x7C
#define REG_ACC_PWR_CTRL            0x7D
#define REG_ACC_SOFTRESET           0x7E

uint8_t acc_range = ACC_RANGE_6G;
// full scale = 1.5 * 2^(range+1) g, so ±3g at range 0; result in mg
float acc_scale = 1000.0f * 1.5f * (float)(2 << ACC_RANGE_6G) / 32768.0f;

static uint8_t acc_read_reg(uint8_t reg) {
    spi_transaction_t t = {
        .length = 24,                 // total bits clocked: 8 (address) + 16 (data)
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
        .tx_data = { reg | 0x80, 0x00, 0x00 },  // bit 7 = read; Read doesn't need any data.
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(bmi088_acc, &t));
    return t.rx_data[2];              //[0] = junk, [1] = dummy, [2] = data
}

static void acc_write_reg(uint8_t reg, uint8_t data) {
    spi_transaction_t t = {
        .length = 16,                 // total bits clocked: 8 (address) + 8 (data)
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
        .tx_data = { reg , data },
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(bmi088_acc, &t));
}

//gets linear acceleration data: x in data[0], y in data[1], z in data[2]
void acc_read(float *data) {
    uint8_t tx[8] = { REG_ACC_DATA_START | 0x80 };  // rest zero-filled
    uint8_t rx[8] = {0};

    spi_transaction_t t = {
        .length = 64,        // 8 address + 8 dummy + 48 data = 64 bits
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(bmi088_acc, &t));

    // rx[0] = junk, rx[1] = dummy, rx[2..7] = X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB
    int16_t raw_x = (int16_t)((rx[3] << 8) | rx[2]);
    int16_t raw_y = (int16_t)((rx[5] << 8) | rx[4]);
    int16_t raw_z = (int16_t)((rx[7] << 8) | rx[6]);

    data[0] = raw_x * acc_scale;
    data[1] = raw_y * acc_scale;
    data[2] = raw_z * acc_scale;
}

esp_err_t acc_set_range(acc_range_t range) {
    //set range
    acc_write_reg(REG_ACC_RANGE, range);
    //verify range set
    uint8_t rd_range = acc_read_reg(REG_ACC_RANGE);
    //set our internal scaling algorithm with the returned range
    // full scale = 1.5 * 2^(range+1) g, so ±3g at range 0; result in mg
    float acc_scale = 1000.0f * 1.5f * (float)(2 << rd_range) / 32768.0f;

    if (rd_range == range) return ESP_OK;
    return ESP_FAIL;
}

static void acc_init(bmi088_config_t* cfg) {
    //reset
    acc_write_reg(REG_ACC_SOFTRESET, ACC_SOFTRESET_CMD);
    vTaskDelay(pdMS_TO_TICKS(2));
    //dummy read to trigger SPI mode
    acc_read_reg(0x00);
    vTaskDelay(pdMS_TO_TICKS(50));
    //check for id
    uint8_t acc_id = acc_read_reg(REG_ACC_CHIP_ID);
    if (acc_id != ACC_ID) ESP_LOGE(TAG, "Accelerometer ID reported as 0x%02X (expected 0x%02X)", acc_id, ACC_ID);

    //set values
    acc_write_reg(REG_ACC_CONF, (cfg->acc_bwp << 4) | (cfg->acc_odr & 0x0F));
    acc_set_range(cfg->acc_range & 0x03);
    acc_write_reg(REG_ACC_FIFO_DOWNS, (cfg->acc_fifo_downs << 4) | 0x80);
    acc_write_reg(REG_ACC_FIFO_WTM_0, cfg->acc_fifo_wtm & 0xFF);
    acc_write_reg(REG_ACC_FIFO_WTM_1, (cfg->acc_fifo_wtm >> 8) & 0x1F);
    acc_write_reg(REG_ACC_FIFO_CONFIG_0, ((cfg->acc_fifo_mode & 0x01) << 1) | 0x02);
    acc_write_reg(REG_ACC_FIFO_CONFIG_1, ((cfg->acc_fifo_en & 0x01) << 6) | 0x10);

    //enable accelerometer
    acc_write_reg(REG_ACC_PWR_CONF, ACC_PWR_CONF_ACTIVE);
    acc_write_reg(REG_ACC_PWR_CTRL, ACC_PWR_CTRL_ON);

    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "Accelerometer initialized");
}

/******************GYROSCOPE**********************/

//register values
#define GYRO_ID                     0x0F
#define GYRO_LPM1_NORMAL            0x00
#define GYRO_LPM1_SUSPEND           0x80
#define GYRO_LPM1_DEEP_SUSPEND      0x20
#define GYRO_SOFTRESET_CMD          0xB6

//register names
#define REG_GYRO_CHIP_ID            0x00   // reset: 0x0F
#define REG_GYRO_DATA_START         0x02   // X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB (0x02-0x07)
#define REG_GYRO_RATE_X_LSB         0x02
#define REG_GYRO_RATE_X_MSB         0x03
#define REG_GYRO_RATE_Y_LSB         0x04
#define REG_GYRO_RATE_Y_MSB         0x05
#define REG_GYRO_RATE_Z_LSB         0x06
#define REG_GYRO_RATE_Z_MSB         0x07
#define REG_GYRO_INT_STAT_1         0x0A
#define REG_GYRO_FIFO_STATUS        0x0E
#define REG_GYRO_RANGE              0x0F
#define REG_GYRO_BANDWIDTH          0x10
#define REG_GYRO_LPM1               0x11
#define REG_GYRO_SOFTRESET          0x14
#define REG_GYRO_INT_CTRL           0x15
#define REG_GYRO_INT3_INT4_IO_CONF  0x16
#define REG_GYRO_INT3_INT4_IO_MAP   0x18
#define REG_GYRO_FIFO_WM_ENABLE     0x1E
#define REG_GYRO_FIFO_EXT_INT_S     0x34
#define REG_GYRO_SELF_TEST          0x3C
#define REG_GYRO_FIFO_CONFIG_0      0x3D
#define REG_GYRO_FIFO_CONFIG_1      0x3E
#define REG_GYRO_FIFO_DATA          0x3F

static uint8_t gyro_read_reg(uint8_t reg) {
    spi_transaction_t t = {
        .length = 16,                 // total bits clocked: 8 (address) + 8 (data)
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
        .tx_data = { reg | 0x80, 0x00 },  // bit 7 = read; Read doesn't need any data.
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(bmi088_gyro, &t));
    return t.rx_data[1];              //[0] = junk, [1] = data
}

static void gyro_write_reg(uint8_t reg, uint8_t data) {
    spi_transaction_t t = {
        .length = 16,                 // total bits clocked: 8 (address) + 8 (data)
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
        .tx_data = { reg , data },
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(bmi088_gyro, &t));
}

/******************SHARED**********************/

static void spi_init(int spi_clock_speed_hz) {
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    //add accelerometer to SPI bus
    spi_device_interface_config_t acc_cfg = {
        .clock_speed_hz = spi_clock_speed_hz,
        .mode = 0,                          // CPOL/CPHA = 00
        .spics_io_num = PIN_CS_ACC,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST, &acc_cfg, &bmi088_acc));

    //add gyroscope to SPI bus
    spi_device_interface_config_t gyro_cfg = {
        .clock_speed_hz = spi_clock_speed_hz,
        .mode = 0,                          // CPOL/CPHA = 00
        .spics_io_num = PIN_CS_GYRO,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST, &gyro_cfg, &bmi088_gyro));
}

void bmi088_init(bmi088_config_t* config) {
    //initialize spi bus
    spi_init(config->spi_clock_speed_hz);
    
    acc_init(config);
    // dummy read to latch SPI mode; ignore result
    //acc_read_reg(0x00);

    //verify id registers of each device
    uint8_t gyro_id = gyro_read_reg(REG_GYRO_CHIP_ID);
    if (gyro_id != GYRO_ID) ESP_LOGE(TAG, "Gyroscope ID reported as 0x%02X (expected 0x%02X)", gyro_id, GYRO_ID);

    

    ESP_LOGI(TAG, "Gyroscope initialized");
}