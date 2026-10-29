#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>


/*
 * ==============================================================
 * PHYSICAL ROBOT CONSTANTS
 * ==============================================================
 *
 * Measured values:
 *
 * Wheel diameter:
 * 6 cm = 0.060 m
 *
 * Robot centre -> each wheel:
 * 5.5 cm
 *
 * Therefore total wheelbase:
 * 11 cm = 0.110 m
 *
 * Encoder calibration:
 *
 * Left:
 * 100 counts / 10 revolutions
 * = 10 counts/revolution
 *
 * Right:
 * 110 counts / 10 revolutions
 * = 11 counts/revolution
 */

#define ODOMETRY_WHEEL_DIAMETER_M     0.060f
#define ODOMETRY_WHEEL_BASE_M         0.110f

#define ODOMETRY_LEFT_CPR             10.0f
#define ODOMETRY_RIGHT_CPR            11.0f


/*
 * ==============================================================
 * ODOMETRY STATE
 * ==============================================================
 */

typedef struct
{
    /*
     * Robot position in metres.
     */
    float x_m;
    float y_m;


    /*
     * Robot heading.
     */
    float heading_rad;
    float heading_deg;


    /*
     * Total forward distance travelled.
     */
    float total_distance_m;


    /*
     * Latest individual wheel distances.
     */
    float left_distance_m;
    float right_distance_m;


    /*
     * Heading calculated only from encoders.
     *
     * Useful for comparing encoder heading
     * against the gyro-based heading.
     */
    float encoder_heading_rad;
    float encoder_heading_deg;

} OdometryState;


/*
 * Reset odometry to:
 *
 * x = 0
 * y = 0
 * heading = 0
 */
void Odometry_Init(void);


/*
 * Update odometry.
 *
 * left_counts:
 * encoder counts since previous update
 *
 * right_counts:
 * encoder counts since previous update
 *
 * gyro_z_dps:
 * filtered Z-axis gyro angular velocity
 *
 * dt_s:
 * control-loop period in seconds
 */
void Odometry_Update(int16_t left_counts,
                     int16_t right_counts,
                     float gyro_z_dps,
                     float dt_s);


/*
 * Get latest odometry state.
 */
const OdometryState *Odometry_GetState(void);


#endif
