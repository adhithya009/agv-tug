#include "odometry.h"

#include <math.h>


/*
 * ==============================================================
 * CONSTANTS
 * ==============================================================
 */

#define ODOMETRY_PI              3.14159265358979323846f

#define DEG_TO_RAD               (ODOMETRY_PI / 180.0f)
#define RAD_TO_DEG               (180.0f / ODOMETRY_PI)


/*
 * ==============================================================
 * INTERNAL STATE
 * ==============================================================
 */

static OdometryState state;


/*
 * ==============================================================
 * ANGLE WRAPPING
 * ==============================================================
 *
 * Keep heading between:
 *
 * -pi and +pi
 */

static float Odometry_WrapAngle(float angle)
{
    while (angle > ODOMETRY_PI)
    {
        angle -=
            2.0f * ODOMETRY_PI;
    }


    while (angle < -ODOMETRY_PI)
    {
        angle +=
            2.0f * ODOMETRY_PI;
    }


    return angle;
}


/*
 * ==============================================================
 * INITIALIZATION
 * ==============================================================
 */

void Odometry_Init(void)
{
    state.x_m = 0.0f;

    state.y_m = 0.0f;

    state.heading_rad = 0.0f;

    state.heading_deg = 0.0f;

    state.total_distance_m = 0.0f;

    state.left_distance_m = 0.0f;

    state.right_distance_m = 0.0f;

    state.encoder_heading_rad = 0.0f;

    state.encoder_heading_deg = 0.0f;
}


/*
 * ==============================================================
 * ODOMETRY UPDATE
 * ==============================================================
 */

void Odometry_Update(int16_t left_counts,
                     int16_t right_counts,
                     float gyro_z_dps,
                     float dt_s)
{
    float wheel_circumference;

    float left_distance;

    float right_distance;

    float centre_distance;

    float encoder_delta_theta;

    float gyro_delta_theta;

    float heading_mid;


    /*
     * ----------------------------------------------------------
     * WHEEL CIRCUMFERENCE
     * ----------------------------------------------------------
     *
     * circumference = pi * diameter
     */

    wheel_circumference =
        ODOMETRY_PI *
        ODOMETRY_WHEEL_DIAMETER_M;


    /*
     * ----------------------------------------------------------
     * LEFT WHEEL DISTANCE
     * ----------------------------------------------------------
     *
     * distance =
     *
     * counts
     * -------
     * CPR
     *
     * x circumference
     */

    left_distance =
        ((float)left_counts /
         ODOMETRY_LEFT_CPR) *
        wheel_circumference;


    /*
     * ----------------------------------------------------------
     * RIGHT WHEEL DISTANCE
     * ----------------------------------------------------------
     */

    right_distance =
        ((float)right_counts /
         ODOMETRY_RIGHT_CPR) *
        wheel_circumference;


    /*
     * Store latest wheel movements.
     */

    state.left_distance_m =
        left_distance;

    state.right_distance_m =
        right_distance;


    /*
     * ----------------------------------------------------------
     * CENTRE OF ROBOT DISTANCE
     * ----------------------------------------------------------
     *
     * Average of left and right wheel movement.
     */

    centre_distance =
        (left_distance +
         right_distance) /
        2.0f;


    /*
     * ----------------------------------------------------------
     * ENCODER-ONLY HEADING
     * ----------------------------------------------------------
     *
     * Differential drive:
     *
     * delta theta =
     *
     * right distance - left distance
     * --------------------------------
     * wheelbase
     */

    encoder_delta_theta =
        (right_distance -
         left_distance) /
        ODOMETRY_WHEEL_BASE_M;


    state.encoder_heading_rad +=
        encoder_delta_theta;


    state.encoder_heading_rad =
        Odometry_WrapAngle(
            state.encoder_heading_rad);


    state.encoder_heading_deg =
        state.encoder_heading_rad *
        RAD_TO_DEG;


    /*
     * ----------------------------------------------------------
     * GYRO HEADING CHANGE
     * ----------------------------------------------------------
     *
     * gyro_z_dps gives:
     *
     * degrees / second
     *
     * Therefore:
     *
     * angle =
     * angular velocity x time
     */

    gyro_delta_theta =
        gyro_z_dps *
        dt_s *
        DEG_TO_RAD;


    /*
     * Use filtered gyro as our main heading source.
     */

    heading_mid =
        state.heading_rad +
        (gyro_delta_theta / 2.0f);


    state.heading_rad +=
        gyro_delta_theta;


    state.heading_rad =
        Odometry_WrapAngle(
            state.heading_rad);


    state.heading_deg =
        state.heading_rad *
        RAD_TO_DEG;


    /*
     * ----------------------------------------------------------
     * UPDATE X POSITION
     * ----------------------------------------------------------
     *
     * Distance projected onto X axis.
     */

    state.x_m +=
        centre_distance *
        cosf(heading_mid);


    /*
     * ----------------------------------------------------------
     * UPDATE Y POSITION
     * ----------------------------------------------------------
     *
     * Distance projected onto Y axis.
     */

    state.y_m +=
        centre_distance *
        sinf(heading_mid);


    /*
     * Total travelled path length.
     */

    state.total_distance_m +=
        fabsf(centre_distance);
}


/*
 * ==============================================================
 * GET STATE
 * ==============================================================
 */

const OdometryState *Odometry_GetState(void)
{
    return &state;
}
