#ifndef AGV_TYPES_H
#define AGV_TYPES_H

#include <stdint.h>

typedef struct {
    uint8_t row;                   // WHY: discrete row used by A*
    uint8_t col;                   // WHY: discrete column used by A*
} GridNode;

typedef struct {
    float x_m;                     // WHY: continuous X position in metres
    float y_m;                     // WHY: continuous Y position in metres
    float heading_rad;             // WHY: internal angular calculations use radians
    float velocity_mps;            // WHY: Kalman/PID need physical velocity
} RobotPose;

typedef struct {
    float left_mps;                // WHY: left wheel measured/target velocity
    float right_mps;               // WHY: right wheel measured/target velocity
} WheelVelocity;

typedef enum {
    AGV_IDLE = 0,
    AGV_PLANNING,
    AGV_NAVIGATING,
    AGV_REPLANNING,
    AGV_ARRIVED,
    AGV_ERROR
} RobotState;

#endif
