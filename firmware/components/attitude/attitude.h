#pragma once

//CONSIDER CHANGING TO 60Hz if there's some LATENCY that you don't like
#define REFRESH_RATE_HZ 30.0f

/*
typedef struct {
    int update_rate_hz;
} att_config_t;
*/

typedef struct {
    float pitch;
    float roll;
} attitude_t;

//starts the attitute reference system - runs an initial calibration - only call once.
esp_err_t attitude_start();

// receive attitude from the queue.
// puts pitch/roll in the out struct.
// returns true if successful. Returns false if no data (meaning you should display that somehow to the user)
bool attitude_get(attitude_t *out, TickType_t timeout);

//requests a calibration from the ARS
bool attitude_request_calibration(TickType_t timeout);