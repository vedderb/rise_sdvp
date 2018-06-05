#ifndef RCONTROLSTATIONCOMM_TYPES_H
#define RCONTROLSTATIONCOMM_TYPES_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    FAULT_CODE_NONE = 0,
    FAULT_CODE_OVER_VOLTAGE,
    FAULT_CODE_UNDER_VOLTAGE,
    FAULT_CODE_DRV8302,
    FAULT_CODE_ABS_OVER_CURRENT,
    FAULT_CODE_OVER_TEMP_FET,
    FAULT_CODE_OVER_TEMP_MOTOR
} mc_fault_code;

typedef struct {
    uint8_t fw_major;
    uint8_t fw_minor;
    double roll;
    double pitch;
    double yaw;
    double accel[3];
    double gyro[3];
    double mag[3];
    double px;
    double py;
    double speed;
    double vin;
    double temp_fet;
    mc_fault_code mc_fault;
    double px_gps;
    double py_gps;
    double ap_goal_px;
    double ap_goal_py;
    double ap_rad;
    int32_t ms_today;
    int16_t ap_route_left;
    double px_uwb;
    double py_uwb;
} CAR_STATE;

typedef struct {
    double px;
    double py;
    double speed;
    int time;
} ROUTE_POINT;

typedef enum {
    RC_MODE_CURRENT = 0,
    RC_MODE_DUTY,
    RC_MODE_PID,
    RC_MODE_CURRENT_BRAKE
} RC_MODE;

#endif // RCONTROLSTATIONCOMM_TYPES_H
