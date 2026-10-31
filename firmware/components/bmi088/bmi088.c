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

uint8_t tx_buff[1026] = {0};//must all be 0's except for the first index [0]
uint8_t rx_buff[1026] = {0};

/******************ACCELEROMETER**********************/

//register values
#define ACC_ID                      0x1E
#define ACC_PWR_CONF_ACTIVE         0x00
#define ACC_PWR_CONF_SUSPEND        0x03
#define ACC_PWR_CTRL_ON             0x04
#define ACC_PWR_CTRL_OFF            0x00
#define ACC_SOFTRESET_CMD           0xB6
#define ACC_FIFO_RESET_CMD          0xB0   // written to ACC_SOFTRESET

//fifo headers
#define ACC_FIFO_HEADER_MASK        0xFC   //masks out last 2 bits which are reserved
#define ACC_FIFO_HEADER_DATA        0x84
#define ACC_FIFO_HEADER_SKIP        0x40
#define ACC_FIFO_HEADER_SENS_TIME   0x44
#define ACC_FIFO_HEADER_CONF        0x48
#define ACC_FIFO_HEADER_SENS_DROP   0x50

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

uint8_t acc_read_reg(uint8_t reg) {
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

//write reg and verify
static esp_err_t acc_write_reg_v(uint8_t reg, uint8_t data) {
    acc_write_reg(reg, data);
    uint8_t read = acc_read_reg(reg);
    if (data != read) {
        ESP_LOGE(TAG, "ACC failed to write 0x%X to 0x%X. Register held 0x%X instead.", data, reg, read);
        return ESP_FAIL;
    }
    return ESP_OK;
}

//reads 2 values from data buffer (current address and address+1)
//returns float in mg
static float acc_raw_to_float(uint8_t* data) {
    return acc_scale * (int16_t)((data[1] << 8) | data[0]);//expand to 16 bits, then cast to float
}

//gets linear acceleration data: x in data[0], y in data[1], z in data[2]
void acc_read(float *data) {
    tx_buff[0] = REG_ACC_DATA_START | 0x80;

    spi_transaction_t t = {
        .length = 64,        // 8 address + 8 dummy + 48 data = 64 bits
        .tx_buffer = tx_buff,
        .rx_buffer = rx_buff,
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(bmi088_acc, &t));

    // rx[0] = junk, rx[1] = dummy, rx[2..7] = X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB
    data[0] = acc_raw_to_float(&rx_buff[2]);
    data[1] = acc_raw_to_float(&rx_buff[4]);
    data[2] = acc_raw_to_float(&rx_buff[6]);
}

//fills data array
uint16_t acc_read_fifo(float* data, uint16_t max_data) {//fifo holds up to 1024 bytes. 
    //get fifo size
    //uint16_t fifo_size = acc_read_reg(REG_ACC_FIFO_LENGTH_0) | ((acc_read_reg(REG_ACC_FIFO_LENGTH_1) & 0x3F) << 8);
    tx_buff[0] = REG_ACC_FIFO_LENGTH_0 | 0x80;
    spi_transaction_t t_read_len = {
        .length = 8 + 8 + (2*8),        // 8 address + 8 dummy + (2 fifo_size(bytes) * 8)
        .tx_buffer = tx_buff,
        .rx_buffer = rx_buff,
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(bmi088_acc, &t_read_len));
    uint16_t fifo_size = (uint16_t)rx_buff[2] | (uint16_t)((rx_buff[3] & 0x3F) << 8);
    
    //verify that we have at least a byte to get
    if (fifo_size == 0) return 0;

    //burst read out all frames
    tx_buff[0] = REG_ACC_FIFO_DATA | 0x80;
    spi_transaction_t t_read = {
        .length = 8 + 8 + (fifo_size*8),        // 8 address + 8 dummy + (fifo_size(bytes) * 8)//reuse previous t struct.
        .tx_buffer = tx_buff,
        .rx_buffer = rx_buff,
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(bmi088_acc, &t_read));

    // rx_buff[0] = junk, rx_buff[1] = dummy, rx[2..fifo_size+2] = fifo data
    uint16_t i = 2;
    size_t j = 0;
    while (i < fifo_size+2) {
        switch (rx_buff[i] & ACC_FIFO_HEADER_MASK) {
            case ACC_FIFO_HEADER_DATA:
                //process raw byte data.
                if (j+2 < max_data) {//verify data will fit.
                    data[j]   = acc_raw_to_float(&rx_buff[i+1]);
                    data[j+1] = acc_raw_to_float(&rx_buff[i+3]);
                    data[j+2] = acc_raw_to_float(&rx_buff[i+5]);
                }
                j += 3;
                i += 7;
                break;
            case ACC_FIFO_HEADER_SKIP:
                //no action
                i += 2;
                break;
            case ACC_FIFO_HEADER_SENS_TIME:
                //fifo was emptied; shouldn't be reached
                i += 2;
                break;
            case ACC_FIFO_HEADER_CONF:
                //no action
                i += 2;
                break;
            case ACC_FIFO_HEADER_SENS_DROP:
                //no action
                i += 2;
                break;
            default:
                i += 1;//failsafe iterate to avoid infinite loop
                break;
        }
    }
    if (j > max_data) j = max_data;
    return j;//j holds number of elements populated in data

}

esp_err_t acc_set_range(acc_range_t range) {
    //set range
    acc_write_reg(REG_ACC_RANGE, range);
    //verify range set
    uint8_t rd_range = acc_read_reg(REG_ACC_RANGE);
    //set our internal scaling algorithm with the returned range
    // full scale = 1.5 * 2^(range+1) g, so ±3g at range 0; result in mg
    acc_scale = 1000.0f * 1.5f * (float)(2 << rd_range) / 32768.0f;

    if (rd_range == range) return ESP_OK;
    return ESP_FAIL;
}

static void acc_sleep() {
    //disable accelerometer
    acc_write_reg(REG_ACC_PWR_CTRL, ACC_PWR_CTRL_OFF);//turn accelerometer off
    acc_write_reg(REG_ACC_PWR_CONF, ACC_PWR_CONF_SUSPEND);//put in suspend mode
}

static void acc_init(bmi088_config_t* cfg) {
    // 1. Dummy read first so SPI mode is latched BEFORE any write
    acc_read_reg(0x00);
    vTaskDelay(pdMS_TO_TICKS(10));

    // 2. Soft reset, then dummy read again (reset returns the chip to I2C mode)
    acc_write_reg(REG_ACC_SOFTRESET, ACC_SOFTRESET_CMD);
    vTaskDelay(pdMS_TO_TICKS(50));
    acc_read_reg(0x00);
    vTaskDelay(pdMS_TO_TICKS(10));

    // 3. Check ID
    uint8_t acc_id = acc_read_reg(REG_ACC_CHIP_ID);
    if (acc_id != ACC_ID) {
        ESP_LOGE(TAG, "Accel ID 0x%02X (expected 0x%02X)", acc_id, ACC_ID);
    }

    // 4. Leave suspend, then enable the sensor, with delays between
    acc_write_reg(REG_ACC_PWR_CONF, ACC_PWR_CONF_ACTIVE);   // 0x00
    vTaskDelay(pdMS_TO_TICKS(10));
    acc_write_reg(REG_ACC_PWR_CTRL, ACC_PWR_CTRL_ON);       // 0x04
    vTaskDelay(pdMS_TO_TICKS(50));

    // 5. Set values
    acc_write_reg_v(REG_ACC_CONF, (cfg->acc_bwp << 4) | (cfg->acc_odr & 0x0F));
    acc_set_range(cfg->acc_range & 0x03);
    acc_write_reg_v(REG_ACC_FIFO_DOWNS, (cfg->acc_fifo_downs << 4) | 0x80);
    acc_write_reg_v(REG_ACC_FIFO_WTM_0, cfg->acc_fifo_wtm & 0xFF);
    acc_write_reg_v(REG_ACC_FIFO_WTM_1, (cfg->acc_fifo_wtm >> 8) & 0x1F);
    acc_write_reg_v(REG_ACC_FIFO_CONFIG_0, ((cfg->acc_fifo_mode & 0x01) << 1) | 0x02);
    acc_write_reg_v(REG_ACC_FIFO_CONFIG_1, ((cfg->acc_fifo_en & 0x01) << 6) | 0x10);

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

float gyro_scale = 0;

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

//write reg and verify
static esp_err_t gyro_write_reg_v(uint8_t reg, uint8_t data) {
    gyro_write_reg(reg, data);
    vTaskDelay(pdMS_TO_TICKS(10));
    uint8_t read = gyro_read_reg(reg);
    if (data != read) {
        ESP_LOGE(TAG, "GYRO failed to write 0x%X to 0x%X. Register held 0x%X instead.", data, reg, read);
        return ESP_FAIL;
    }
    return ESP_OK;
}

//returns float in degrees per second
static float gyro_raw_to_float(uint8_t* data) {
    return gyro_scale * (int16_t)((data[1] << 8) | data[0]);//expand to 16 bits, then cast to float
}

//fills data array
uint16_t gyro_read_fifo(float* data, uint16_t max_data) {//fifo holds up to 1024 bytes. 
    //get fifo size
    uint16_t fifo_size = (gyro_read_reg(REG_GYRO_FIFO_STATUS) & 0x7F) * 6;
    
    //verify that we have at least a byte to get
    if (fifo_size == 0) return 0;

    //burst read out all frames
    tx_buff[0] = REG_GYRO_FIFO_DATA | 0x80;
    spi_transaction_t t_read = {
        .length = 8 + (fifo_size*8),        // 8 address + (fifo_size(bytes) * 8)//reuse previous t struct.
        .tx_buffer = tx_buff,
        .rx_buffer = rx_buff,
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(bmi088_gyro, &t_read));

    // rx_buff[0] = junk, rx[2..fifo_size+2] = fifo data
    size_t j = 0;
    for (size_t i = 1; i < fifo_size + 1; i+=6) {
        if (j+2 < max_data) {//verify data will fit.
            data[j]   = gyro_raw_to_float(&rx_buff[i]);
            data[j+1] = gyro_raw_to_float(&rx_buff[i+2]);
            data[j+2] = gyro_raw_to_float(&rx_buff[i+4]);
        }
        j+=3;
    }

    if (j > max_data) j = max_data;
    return j;//j holds number of elements populated in data
}

esp_err_t gyro_set_range(gyro_range_t range) {
    // full scale = 1.5 * 2^(range+1) g, so ±3g at range 0; result in mg
    switch (range)
    {
    case GYRO_RANGE_2000DPS:
        gyro_scale = 2000.0f;
        break;
    case GYRO_RANGE_1000DPS:
        gyro_scale = 1000.0f;
        break;
    case GYRO_RANGE_500DPS:
        gyro_scale = 500.0f;
        break;
    case GYRO_RANGE_250DPS:
        gyro_scale = 250.0f;
        break;
    case GYRO_RANGE_125DPS:
        gyro_scale = 125.0f;
        break;
    }
    gyro_scale /= 32767.0F;
    //set range
    return gyro_write_reg_v(REG_GYRO_RANGE, range);
}

static void gyro_sleep() {
    //suspend gyro
    gyro_write_reg_v(REG_GYRO_LPM1, GYRO_LPM1_SUSPEND);
}

static void gyro_init(bmi088_config_t* cfg) {
    // 1. Soft reset, then dummy read again (reset returns the chip to I2C mode)
    gyro_write_reg(REG_GYRO_SOFTRESET, GYRO_SOFTRESET_CMD);
    vTaskDelay(pdMS_TO_TICKS(50));

    // 2. Check ID
    uint8_t gyro_id = gyro_read_reg(REG_GYRO_CHIP_ID);
    if (gyro_id != GYRO_ID) {
        ESP_LOGE(TAG, "Gyro ID 0x%02X (expected 0x%02X)", gyro_id, GYRO_ID);
    }

    // 4. Leave suspend (just in case it somehow didn't reset properly)
    gyro_write_reg(REG_GYRO_LPM1, GYRO_LPM1_NORMAL);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // 5. Write settings registers
    //vTaskDelay(pdMS_TO_TICKS(50));
    gyro_set_range(cfg->gyro_range);
    //vTaskDelay(pdMS_TO_TICKS(50));
    gyro_write_reg_v(REG_GYRO_BANDWIDTH, cfg->gyro_bw | 0x80);
    //vTaskDelay(pdMS_TO_TICKS(50));
    gyro_write_reg_v(REG_GYRO_FIFO_CONFIG_1, cfg->gyro_fifo_mode);

    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "Gyroscope initialized");

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

void bmi088_sleep() {
    acc_sleep();
    gyro_sleep();
}

void bmi088_init(bmi088_config_t* config) {
    //initialize spi bus
    spi_init(config->spi_clock_speed_hz);
    
    //initialize both devices
    acc_init(config);
    gyro_init(config);
}