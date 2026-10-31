#pragma once

#define PIN_MOSI    12//default 11
#define PIN_MISO    13//default 13
#define PIN_SCLK    11//default 12
#define PIN_CS_ACC  9//default 10
#define PIN_CS_GYRO 10//default 10

#define SPI_HOST   SPI2_HOST

typedef enum {
    ACC_BWP_NORM = 0x0A,
    ACC_BWP_OSR2 = 0x09,
    ACC_BWP_OSR4 = 0x08,
} acc_bwp_t;

typedef enum {
    ACC_ODR_12_5_HZ = 0x05,
    ACC_ODR_25_HZ   = 0x06,
    ACC_ODR_50_HZ   = 0x07,
    ACC_ODR_100_HZ  = 0x08,
    ACC_ODR_200_HZ  = 0x09,
    ACC_ODR_400_HZ  = 0x0A,
    ACC_ODR_800_HZ  = 0x0B,
    ACC_ODR_1600_HZ = 0x0C,
    
} acc_odr_t;

typedef enum {
    ACC_RANGE_3G  = 0x00,
    ACC_RANGE_6G  = 0x01,//default range
    ACC_RANGE_12G = 0x02,
    ACC_RANGE_24G = 0x03,
} acc_range_t;

typedef enum {
    ACC_FIFO_MODE_STREAM = 0x00,
    ACC_FIFO_MODE_FIFO   = 0x01,
} acc_fifo_mode_t;

typedef enum {
    GYRO_RANGE_2000DPS = 0x00,
    GYRO_RANGE_1000DPS = 0x01,
    GYRO_RANGE_500DPS  = 0x02,
    GYRO_RANGE_250DPS  = 0x03,
    GYRO_RANGE_125DPS  = 0x04,
} gyro_range_t;

typedef enum {
    GYRO_BW_532 = 0x00,
    GYRO_BW_230 = 0x01,
    GYRO_BW_116 = 0x02,
    GYRO_BW_47  = 0x03,
    GYRO_BW_23  = 0x04,
    GYRO_BW_12  = 0x05,
    GYRO_BW_64  = 0x06,
    GYRO_BW_32  = 0x07,
} gyro_bw_t;

typedef enum {
    GYRO_FIFO_MODE_STREAM = 0x80,
    GYRO_FIFO_MODE_FIFO   = 0x40,
} gyro_fifo_mode_t;

typedef struct {
    int spi_clock_speed_hz;
    
    acc_bwp_t acc_bwp;//bandwith low-pass setting. (oversampling+filter)
    acc_odr_t acc_odr;//output data rate
    acc_range_t acc_range;//accelerometer range
    uint8_t acc_fifo_en;//1 bit enable fifo
    acc_fifo_mode_t acc_fifo_mode;
    uint8_t acc_fifo_downs;//3 bit fifo downsample
    uint16_t acc_fifo_wtm;//13 bit watermark level for interrupts

    gyro_range_t gyro_range;//max and min values in degrees per second
    gyro_bw_t gyro_bw;//bandwidth or output data rate setings. See datasheet.
    gyro_fifo_mode_t gyro_fifo_mode;
} bmi088_config_t;

#define BMI088_DEFAULT_CONFIG() { \
    .spi_clock_speed_hz = 5 * 1000 * 1000, \
    .acc_bwp = ACC_BWP_NORM, \
    .acc_odr = ACC_ODR_100_HZ,\
    .acc_range = ACC_RANGE_6G,\
    .acc_fifo_en = 0x00,\
    .acc_fifo_mode = ACC_FIFO_MODE_STREAM,\
    .acc_fifo_downs = 0x00,\
    .acc_fifo_wtm = 0x0200,\
    .gyro_range = GYRO_RANGE_2000DPS,\
    .gyro_bw = GYRO_BW_532,\
    .gyro_fifo_mode = GYRO_FIFO_MODE_STREAM,\
}

void bmi088_sleep();//enables sleep low power. To enable, run init with your previous configuration settings.
void bmi088_init(bmi088_config_t* config);//enable the bmi088.

uint16_t acc_read_fifo(float* data, uint16_t max_data);
uint16_t gyro_read_fifo(float* data, uint16_t max_data);

