#pragma once

#include <stdbool.h>

// TE Connectivity MS4525DO / 4525DO-DS5AI001DP
// Differential ±1 PSI, I2C address 0x28, output type A (10-90%)
//
// Wiring (ESP32-S3 DevKitC-1):
//   Pin 1 GND    -> GND
//   Pin 2 SUPPLY -> 5 V
//   Pin 3 SDA    -> GPIO8
//   Pin 4 SCL    -> GPIO9
//   Pin 5 INT    -> leave open
//
// I2C is open-drain. Prefer external 4.7k pull-ups to 3.3 V on SDA/SCL.
// The sensor itself is powered from 5 V.

#define AIRSPEED_I2C_PORT   I2C_NUM_0
#define AIRSPEED_SDA_PIN    8
#define AIRSPEED_SCL_PIN    9
#define AIRSPEED_I2C_ADDR   0x28
#define AIRSPEED_I2C_FREQ   100000  // 100 kHz

void airspeed_init(void);

/**
 * Read differential pressure (Port1 - Port2) in Pascals.
 * Positive when Port 1 > Port 2.
 * Also returns die temperature in °C.
 * Returns true on success.
 */
bool airspeed_read_diff_pa(float *pressure_pa, float *temp_c);
