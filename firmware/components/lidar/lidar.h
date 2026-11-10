#pragma once

#include <stdbool.h>
#include <stdint.h>

// LightWare LW20/C on UART0 (ESP32-S3 DevKitC-1 pins)
// LiDAR Yellow (TXD) -> ESP32 GPIO44 (U0RXD)
// LiDAR White  (RXD) -> ESP32 GPIO43 (U0TXD)
// LiDAR Red    (VIN) -> 5V
// LiDAR Black  (GND) -> GND
// Logic is 3.3 V TTL (5 V tolerant)

#define LIDAR_UART_NUM   UART_NUM_0
#define LIDAR_TX_PIN     43
#define LIDAR_RX_PIN     44
#define LIDAR_BAUD       115200

void lidar_init(void);

/**
 * Request one slant-range reading in meters.
 * Returns true and writes *meters on success.
 * Uses ASCII command "?L\r\n".
 */
bool lidar_read_slant_m(float *meters);
