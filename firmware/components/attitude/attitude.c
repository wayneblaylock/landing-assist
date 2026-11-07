//The following open-source code is used from Sebastian Madgwick, with modifications to support application-specific needs.
//also working on fixing a latent divide by zero possibility in Madgwick's original code.

#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h" 
#include "freertos/semphr.h" 

#include "bmi088.h"

#include "attitude.h"

#define TASK_PERIOD pdMS_TO_TICKS(1000.0f / REFRESH_RATE_HZ)
#define TAG "attitude"

#define DEG_TO_RAD 0.0174533f
#define RAD_TO_DEG 57.2958f
#define MAX_FIFO_DATA 450
#define STACK_DEPTH 2048    // Define the size of the task stack
StackType_t xAttStackTask[STACK_DEPTH];    // Stack space used by Task 1, size is STACK_DEPTH
StaticTask_t xAttTaskBuffer;               // Task control

// System constants
float deltat = 0.001f;
float gyroMeasError;
float beta;

// Global system variables
//float a_x, a_y, a_z; // accelerometer measurements
//float w_x, w_y, w_z; // gyroscope measurements in rad/s
float SEq_1 = 1.0f, SEq_2 = 0.0f, SEq_3 = 0.0f, SEq_4 = 0.0f; // estimated orientation quaternion elements with initial conditions

//gyro bias. Must be added to gyroscope measurements before putting data in filter.
float gb_x = 0.0f; float gb_y = 0.0f; float gb_z = 0.0f;

static QueueHandle_t s_att_queue;//the que holds the latest value of pitch/roll
static TaskHandle_t  s_att_task;//the task handle holds the attitude_task() task
static SemaphoreHandle_t s_cal_done;//semaphore blocks the main task while calibration occurs

static void set_sample_period(float sample_period) {
    deltat = sample_period;// sampling period in seconds
}

static void set_gyro_error(float gyro_error_degps) {
    gyroMeasError = 3.14159265358979f * (gyro_error_degps / 180.0f);// gyroscope measurement error in rad/s
    beta = sqrt(3.0f / 4.0f) * gyroMeasError;// compute beta
}

//note that we use the FLU convention for everything, but here we have an exception for pitch, since we like positive pitch to be nose up.
attitude_t get_pitch_roll() {
    attitude_t att;
    //roll = atan2f(2*(qw*qx + qy*qz), 1 - 2*(qx*qx+qy*qy))
    //pitch = asinf(2*(qw*qy - qz*qx))
    
    att.roll = atan2f(2.0f*(SEq_1*SEq_2 + SEq_3*SEq_4),
                        1.0f - 2.0f*(SEq_2*SEq_2 + SEq_3*SEq_3)) * RAD_TO_DEG;

    float sinp = 2.0f * (SEq_1*SEq_3 - SEq_4*SEq_2);
    if (sinp > 1.0f)  sinp = 1.0f;//clamp
    if (sinp < -1.0f) sinp = -1.0f;
    att.pitch = -asinf(sinp) * RAD_TO_DEG;

    return att;
}

//madgwick filter. Each successive call should include samples taken deltat apart
static void filter_update(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z)
{
    // Local system variables
    float norm; // vector norm
    float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion derrivative from gyroscopes elements
    float f_1, f_2, f_3; // objective function elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
    // Axulirary variables to avoid reapeated calcualtions
    float halfSEq_1 = 0.5f * SEq_1;
    float halfSEq_2 = 0.5f * SEq_2;
    float halfSEq_3 = 0.5f * SEq_3;
    float halfSEq_4 = 0.5f * SEq_4;

    float twoSEq_1 = 2.0f * SEq_1;
    float twoSEq_2 = 2.0f * SEq_2;
    float twoSEq_3 = 2.0f * SEq_3;

    // Normalise the accelerometer measurement
    norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    if (norm < 1e-6f) return;          // no usable gravity reading
    a_x /= norm;
    a_y /= norm;
    a_z /= norm;

    // Compute the objective function and Jacobian
    f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
    f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
    f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;

    J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
    J_12or23 = 2.0f * SEq_4;
    J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
    J_14or21 = twoSEq_2;
    J_32 = 2.0f * J_14or21; // negated in matrix multiplication
    J_33 = 2.0f * J_11or24; // negated in matrix multiplication

    // Compute the gradient (matrix multiplication)
    SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
    SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
    SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
    SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;

    // Normalise the gradient
    norm = sqrtf(SEqHatDot_1*SEqHatDot_1 + SEqHatDot_2*SEqHatDot_2 + SEqHatDot_3*SEqHatDot_3 + SEqHatDot_4*SEqHatDot_4);
    if (norm > 1e-9f) {
        SEqHatDot_1 /= norm;
        SEqHatDot_2 /= norm;
        SEqHatDot_3 /= norm;
        SEqHatDot_4 /= norm;
    } else {
        // already aligned with gravity: no correction needed
        SEqHatDot_1 = SEqHatDot_2 = SEqHatDot_3 = SEqHatDot_4 = 0.0f;
    }

    // Compute the quaternion derrivative measured by gyroscopes
    SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
    SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
    SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
    SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;

    // Compute then integrate the estimated quaternion derrivative
    SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
    SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
    SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
    SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;

    // Normalise quaternion
    norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
    SEq_1 /= norm;
    SEq_2 /= norm;
    SEq_3 /= norm;
    SEq_4 /= norm;
}

//calibrates the attitude. Run while the gyro is stationary
//removes gyro drift (zeros gyro)
//aligns internal quaternion to level horizon
//sets pitch/roll to level based on offset
//uses buf1 and buf2 to avoid stack usage. Please ensure these can handle at least MAX_FIFO_DATA floats each.
//this should only ever be called form the attitdue_task() funciton
#define CALIBRATION_DURATION_MS 2000.0f
#define CALIBRATION_SAMPLES 60
#define CALIBRATION_SAMPLE_PERIOD CALIBRATION_DURATION_MS / 60
static bool calibrate_attitude(float* buf1, float* buf2) {
    //first get an average acceleration and gyro rates
    float w_x_ave = 0;
    float w_y_ave = 0;
    float w_z_ave = 0;
    float a_x_ave = 0;
    float a_y_ave = 0;
    float a_z_ave = 0;
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        //delay
        vTaskDelay(pdMS_TO_TICKS(CALIBRATION_SAMPLE_PERIOD));
        //read fifo
        int16_t accel_count = acc_read_fifo(buf1, MAX_FIFO_DATA);
        int16_t gyro_count =  gyro_read_fifo(buf2, MAX_FIFO_DATA);
        if (accel_count > 0 && gyro_count > 0) {
            w_x_ave += buf2[0];
            w_y_ave += buf2[1];
            w_z_ave += buf2[2];
            a_x_ave += buf1[0];
            a_y_ave += buf1[1];
            a_z_ave += buf1[2];
        } else {
            ESP_LOGE(TAG, "calibration failed: no FIFO data");
            return false;
        }
    }
    w_x_ave /= CALIBRATION_SAMPLES;
    w_y_ave /= CALIBRATION_SAMPLES;
    w_z_ave /= CALIBRATION_SAMPLES;
    a_x_ave /= CALIBRATION_SAMPLES;
    a_y_ave /= CALIBRATION_SAMPLES;
    a_z_ave /= CALIBRATION_SAMPLES;

    //next, set the gyro bias to opposite the average.
    gb_x = w_x_ave * -DEG_TO_RAD;
    gb_y = w_y_ave * -DEG_TO_RAD;
    gb_z = w_z_ave * -DEG_TO_RAD;

    //next, snap the internal state quaternion to the acceleration vector
    float norm = sqrtf(a_x_ave*a_x_ave + a_y_ave*a_y_ave + a_z_ave*a_z_ave);
    //if (norm < 1e-6f) return;           // bad data, keep identity
    a_x_ave /= norm; a_y_ave /= norm; a_z_ave /= norm;

    if (a_z_ave < -0.9999f) {
        // Upside down: 1 + az -> 0, so use a 180 deg rotation about X to avoid divide by zero
        SEq_1 = 0.0f; SEq_2 = 1.0f; SEq_3 = 0.0f; SEq_4 = 0.0f;
        return false;
    }

    float w = 1.0f + a_z_ave;
    float x = a_y_ave;
    float y = -a_x_ave;
    float z = 0.0f;

    norm = sqrtf(w*w + x*x + y*y + z*z);
    SEq_1 = w / norm;
    SEq_2 = x / norm;
    SEq_3 = y / norm;
    SEq_4 = z / norm;

    //finally, set current pitch/roll as the neutral position
    //TODO: IMPLEMENT*******************
    //this should probably create a rotation matrix that we can apply to pre-rotate out acceleration and rotation values
    //around to get them in terms of the neutral position

    ESP_LOGI(TAG, "calibrated");
    return true;
}

#define ATTITUDE_SAMPLE_PERIOD 1.0f / 200.0f //200 Hz
#define ATTITUDE_GYRO_ERROR 1.0f //error in degrees 5.0 for testing, maybe 0.5 for actual use?
void attitude_task(void* arg) {

    //buffer of many accel and gyro readings
    static float accel[MAX_FIFO_DATA];
    static float gyro[MAX_FIFO_DATA];
    int16_t accel_count, gyro_count;

    set_sample_period(ATTITUDE_SAMPLE_PERIOD);
    set_gyro_error(ATTITUDE_GYRO_ERROR);
    
    //configure and initialize bmi088
    bmi088_config_t bmi088_cfg = BMI088_DEFAULT_CONFIG();

    bmi088_cfg.spi_clock_speed_hz = 8 * 1000 * 1000;//8 MHz
    bmi088_cfg.acc_fifo_en = 0x01; //enable FIFO
    bmi088_cfg.acc_odr = ACC_ODR_200_HZ; //200Hz accelerometer
    bmi088_cfg.gyro_bw = GYRO_BW_64;//200Hz gyroscope
    bmi088_cfg.gyro_range = GYRO_RANGE_500DPS;//500 degrees per second max range gives us a resoluiton of 0.0153 degrees per second (per LSB)

    bmi088_init(&bmi088_cfg);

    //initial calibration
    calibrate_attitude(accel, gyro);

    //task timing variables
    TickType_t last_wake = xTaskGetTickCount();
    BaseType_t was_delayed;

    while (1) {

        // Calibration requested?
        if (ulTaskNotifyTake(pdTRUE, 0) > 0) {
            xQueueReset(s_att_queue); //invalidate attitude queue
            calibrate_attitude(accel, gyro);
            xSemaphoreGive(s_cal_done);
            last_wake = xTaskGetTickCount();   // re-anchor after the ~2 s calibration
        }

        was_delayed = xTaskDelayUntil(&last_wake, TASK_PERIOD);

        //read in all fifo values form accelerometer and gyroscope
        accel_count = acc_read_fifo(accel, MAX_FIFO_DATA);
        gyro_count =  gyro_read_fifo(gyro, MAX_FIFO_DATA);
        if (gyro_count < 3 || accel_count < 3) {
            xQueueReset(s_att_queue); //invalidate attitude queue
            continue;
        }

        //log latest data point from fifo
        //ESP_LOGI(TAG, "BMI088Accelerometer read in %d values from FIFO. Last 3: %.3f %.3f %.3f", accel_count, accel[accel_count-3],accel[accel_count-2],accel[accel_count-1]);
        ESP_LOGI(TAG, "BMI088Gyroscope     read in %d values from FIFO. Last 3: %.3f %.3f %.3f", gyro_count, gyro[gyro_count-3],gyro[gyro_count-2],gyro[gyro_count-1]);

        //we need to run the filter at least once for each gyroscope data point (since these are accumulated).
        //But accelerometer data is less crititcal, so we don't worry about tight alignment, and if there are an excess of gyro points, we extend the acc data.
        for (int16_t i = 0; i < gyro_count; i += 3) {
            int16_t acc_index = (i < accel_count) ? i : accel_count - 3 ;//acc_index tracks i unless i is out of bounds, then it tracks the last point
            //update filter
            filter_update(
                gyro[i  ]*DEG_TO_RAD + gb_x,
                gyro[i+1]*DEG_TO_RAD + gb_y,
                gyro[i+2]*DEG_TO_RAD + gb_z,
                accel[acc_index  ],
                accel[acc_index+1],
                accel[acc_index+2]
            );
        }

        //ESP_LOGI(TAG, "quaternion: %.3f\t%.3f\t%.3f\t%.3f", SEq_1, SEq_2, SEq_3, SEq_4);

        //calculate pitch/roll and push to the attitude queue
        attitude_t attitude = get_pitch_roll();
        xQueueOverwrite(s_att_queue, &attitude);
        //ESP_LOGI(TAG, "Pitch/Roll: %.3f %.3f", attitude.pitch, attitude.roll);
    }
}

//only start once
esp_err_t attitude_start() {
    s_cal_done = xSemaphoreCreateBinary();

    s_att_queue = xQueueCreate(1, sizeof(attitude_t));
    if (s_att_queue == NULL) return ESP_ERR_NO_MEM;

    s_att_task = xTaskCreateStatic(attitude_task, "attitude", STACK_DEPTH, NULL, 1, xAttStackTask, &xAttTaskBuffer);
    if (s_att_task == NULL) return ESP_FAIL;

    return ESP_OK;
}

bool attitude_get(attitude_t *out, TickType_t timeout)
{
    return xQueuePeek(s_att_queue, out, timeout) == pdTRUE;
}

// Blocks the caller until the attitude task finishes recalibrating
bool attitude_request_calibration(TickType_t timeout) {
    xSemaphoreTake(s_cal_done, 0);          // clear any stale signal
    xTaskNotifyGive(s_att_task);
    return xSemaphoreTake(s_cal_done, timeout) == pdTRUE;
}