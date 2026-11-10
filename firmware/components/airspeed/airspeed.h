#pragma once

#include <stdbool.h>
#include <stdint.h>

// Working wiring (SDA/SCL were swapped from the datasheet assumption)
#define AIRSPEED_SDA_PIN    9   // sensor pin 3 → GPIO 9
#define AIRSPEED_SCL_PIN    8   // sensor pin 4 → GPIO 8
#define AIRSPEED_I2C_PORT   I2C_NUM_0
#define AIRSPEED_I2C_ADDR   0x28
#define AIRSPEED_I2C_FREQ   100000

void airspeed_init(void);
bool airspeed_read_diff_pa(float *pressure_pa, float *temp_c);
