#pragma once

#include <stdbool.h>
#include <stdint.h>

// LightWare LW20/C on UART (ESP32-S3 pins)
// LiDAR Yellow (TXD) -> ESP32 GPIO18 (RX)
// LiDAR White  (RXD) -> ESP32 GPIO17 (TX)
// LiDAR Red    (VIN) -> 5V
// LiDAR Black  (GND) -> GND
// Logic is 3.3 V TTL (5 V tolerant)

#define LIDAR_UART_NUM   UART_NUM_0
#define LIDAR_TX_PIN     17
#define LIDAR_RX_PIN     18
#define LIDAR_BAUD       115200

void lidar_init(void);

/**
 * Request one slant-range reading in meters.
 * Returns true and writes *meters on success.
 * Uses ASCII command "?L\r\n".
 */
bool lidar_read_slant_m(float *meters);
