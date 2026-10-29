#include "test_full_navigation.h"

#include <stdint.h>

#include "map.h"
#include "astar.h"
#include "navigation.h"
#include "ir_array.h"
#include "motor.h"
#include "encoder.h"
#include "mpu6050.h"
#include "kalman.h"
#include "odometry.h"
#include "timer_cfg.h"


/*
 * ==============================================================
 * COMPLETE LINE-FOLLOWING NAVIGATION TEST
 * ==============================================================
 *
 * START       = S2
 * DESTINATION = S3
 *
 * Route expected:
 *
 * S2 -> J2 -> J1 -> S5 -> J8 -> S3
 *
 * Robot control:
 *
 * IR + navigation + PID + motors
 *
 * Monitoring:
 *
 * MPU + gyro + Kalman + encoders + odometry
 */


/*
 * ==============================================================
 * SETTINGS
 * ==============================================================
 */

#define FULL_NAV_KP                  3.0f
#define FULL_NAV_KI                  0.0f
#define FULL_NAV_KD                  0.0f

#define FULL_NAV_DT_S                0.010f

/*
 * CHANGED:
 *
 * 70% did not reliably start the left motor.
 */
#define FULL_NAV_BASE_SPEED 68U

/*
 * 2 second stationary startup.
 */
#define FULL_NAV_START_DELAY_TICKS   200U

/*
 * Number of gyro samples used for bias.
 */
#define FULL_NAV_GYRO_CAL_SAMPLES    100U

/*
 * 60 second emergency timeout.
 */
#define FULL_NAV_TIMEOUT_TICKS       6000U

/*
 * Temporary destination confirmation.
 */
#define FULL_NAV_DEST_CONFIRM        3U


/*
 * ==============================================================
 * TEST STATES
 * ==============================================================
 */

#define FULL_NAV_STARTING            0U
#define FULL_NAV_RUNNING             1U
#define FULL_NAV_DESTINATION         2U
#define FULL_NAV_ASTAR_FAILED        3U
#define FULL_NAV_TIMEOUT             4U


/*
 * ==============================================================
 * GENERAL DEBUG
 * ==============================================================
 */

volatile uint8_t debug_fullnav_state = 0U;

volatile uint8_t debug_fullnav_done = 0U;

volatile uint8_t debug_fullnav_start_station = 2U;

volatile uint8_t debug_fullnav_destination_station = 3U;


/*
 * ==============================================================
 * A* DEBUG
 * ==============================================================
 */

volatile uint8_t debug_fullnav_astar_ok = 0U;

volatile uint8_t debug_fullnav_path_length = 0U;

volatile uint8_t
debug_fullnav_path[MAP_NODE_COUNT];

volatile uint8_t debug_fullnav_path_index = 0U;

volatile uint8_t debug_fullnav_target_node =
    MAP_NODE_INVALID;


/*
 * ==============================================================
 * IR DEBUG
 * ==============================================================
 */

volatile uint8_t debug_fullnav_ir_pattern = 0U;


/*
 * ==============================================================
 * TIME DEBUG
 * ==============================================================
 */

volatile uint32_t debug_fullnav_ticks = 0U;

volatile uint32_t debug_fullnav_elapsed_ms = 0U;

volatile uint32_t debug_fullnav_start_delay_ms = 0U;

volatile uint8_t debug_fullnav_destination_confirm = 0U;


/*
 * ==============================================================
 * ENCODER DEBUG
 * ==============================================================
 */

volatile uint16_t debug_fullnav_left_raw = 0U;

volatile uint16_t debug_fullnav_right_raw = 0U;

volatile int16_t debug_fullnav_left_delta = 0;

volatile int16_t debug_fullnav_right_delta = 0;

volatile uint8_t debug_fullnav_encoder_warning = 0U;


/*
 * ==============================================================
 * MPU / KALMAN DEBUG
 * ==============================================================
 */

volatile uint8_t debug_fullnav_mpu_init_ok = 0U;

volatile uint8_t debug_fullnav_mpu_read_ok = 0U;

volatile uint16_t debug_fullnav_gyro_cal_samples = 0U;

volatile float debug_fullnav_gyro_raw_dps = 0.0f;

volatile float debug_fullnav_gyro_bias_dps = 0.0f;

volatile float debug_fullnav_gyro_corrected_dps = 0.0f;

volatile float debug_fullnav_gyro_filtered_dps = 0.0f;

volatile float debug_fullnav_accel_x_g = 0.0f;

volatile float debug_fullnav_accel_y_g = 0.0f;

volatile float debug_fullnav_accel_z_g = 0.0f;


/*
 * ==============================================================
 * ODOMETRY DEBUG
 * ==============================================================
 */

volatile float debug_fullnav_x_m = 0.0f;

volatile float debug_fullnav_y_m = 0.0f;

volatile float debug_fullnav_heading_deg = 0.0f;

volatile float debug_fullnav_encoder_heading_deg = 0.0f;

volatile float debug_fullnav_distance_m = 0.0f;

volatile float debug_fullnav_left_distance_m = 0.0f;

volatile float debug_fullnav_right_distance_m = 0.0f;


/*
 * ==============================================================
 * PRIVATE DATA
 * ==============================================================
 */

static AStarPath full_nav_path;

static MPU6050Data full_nav_mpu;

static Kalman1D full_nav_gyro_filter;


/*
 * ==============================================================
 * CLEAR DEBUG PATH
 * ==============================================================
 */

static void FullNav_ClearDebugPath(void)
{
    for (uint8_t i = 0U;
         i < (uint8_t)MAP_NODE_COUNT;
         i++)
    {
        debug_fullnav_path[i] =
            (uint8_t)MAP_NODE_INVALID;
    }
}


/*
 * ==============================================================
 * COPY PATH FOR DEBUGGING
 * ==============================================================
 */

static void FullNav_CopyDebugPath(void)
{
    debug_fullnav_path_length =
        full_nav_path.length;


    FullNav_ClearDebugPath();


    for (uint8_t i = 0U;
         i < full_nav_path.length;
         i++)
    {
        debug_fullnav_path[i] =
            (uint8_t)
            full_nav_path.nodes[i];
    }
}


/*
 * ==============================================================
 * COPY ODOMETRY DATA
 * ==============================================================
 */

static void FullNav_CopyOdometryDebug(void)
{
    const OdometryState *state =
        Odometry_GetState();


    debug_fullnav_x_m =
        state->x_m;


    debug_fullnav_y_m =
        state->y_m;


    debug_fullnav_heading_deg =
        state->heading_deg;


    debug_fullnav_encoder_heading_deg =
        state->encoder_heading_deg;


    debug_fullnav_distance_m =
        state->total_distance_m;


    debug_fullnav_left_distance_m =
        state->left_distance_m;


    debug_fullnav_right_distance_m =
        state->right_distance_m;
}


/*
 * ==============================================================
 * UPDATE MPU + ENCODER + ODOMETRY MONITORING
 * ==============================================================
 *
 * None of this controls the motors.
 */

static void FullNav_UpdateMonitoring(void)
{
    /*
     * ------------------------------
     * ENCODERS
     * ------------------------------
     */

    Encoder_Update();


    debug_fullnav_left_raw =
        Encoder_GetLeftRaw();


    debug_fullnav_right_raw =
        Encoder_GetRightRaw();


    debug_fullnav_left_delta =
        Encoder_GetLeftDelta();


    debug_fullnav_right_delta =
        Encoder_GetRightDelta();


    /*
     * Detect one obviously dead encoder.
     *
     * Diagnostic only.
     */
    if ((debug_fullnav_left_raw > 20U) &&
        (debug_fullnav_right_raw == 0U))
    {
        debug_fullnav_encoder_warning =
            1U;
    }


    if ((debug_fullnav_right_raw > 20U) &&
        (debug_fullnav_left_raw == 0U))
    {
        debug_fullnav_encoder_warning =
            1U;
    }


    /*
     * ------------------------------
     * MPU
     * ------------------------------
     */

    if (debug_fullnav_mpu_init_ok)
    {
        debug_fullnav_mpu_read_ok =
            MPU6050_Read(
                &full_nav_mpu);


        if (debug_fullnav_mpu_read_ok)
        {
            debug_fullnav_accel_x_g =
                full_nav_mpu.accel_x_g;


            debug_fullnav_accel_y_g =
                full_nav_mpu.accel_y_g;


            debug_fullnav_accel_z_g =
                full_nav_mpu.accel_z_g;


            debug_fullnav_gyro_raw_dps =
                full_nav_mpu.gyro_z_dps;


            debug_fullnav_gyro_corrected_dps =
                debug_fullnav_gyro_raw_dps -
                debug_fullnav_gyro_bias_dps;


            debug_fullnav_gyro_filtered_dps =
                Kalman1D_Update(
                    &full_nav_gyro_filter,
                    debug_fullnav_gyro_corrected_dps);


            /*
             * Monitoring only.
             */
            Odometry_Update(
                debug_fullnav_left_delta,
                debug_fullnav_right_delta,
                debug_fullnav_gyro_filtered_dps,
                FULL_NAV_DT_S);
        }
    }


    FullNav_CopyOdometryDebug();
}


/*
 * ==============================================================
 * STOP ROBOT
 * ==============================================================
 */

static void FullNav_Stop(
    uint8_t reason)
{
    Navigation_Stop();


    Motor_Stop();


    debug_fullnav_state =
        reason;


    debug_fullnav_done =
        1U;
}


/*
 * ==============================================================
 * FULL NAVIGATION TEST
 * ==============================================================
 */

void Test_FullNavigation(void)
{
    /*
     * Safe startup.
     */
    Motor_Stop();


    /*
     * Build graph.
     */
    Map_Init();


    /*
     * Reset motion monitoring.
     */
    Encoder_Reset();


    Odometry_Init();


    /*
     * Initialize IR navigation.
     */
    Navigation_Init(
        FULL_NAV_KP,
        FULL_NAV_KI,
        FULL_NAV_KD,
        FULL_NAV_DT_S,
        FULL_NAV_BASE_SPEED);


    /*
     * MPU is optional for navigation.
     *
     * Failure does NOT stop the robot.
     */
    debug_fullnav_mpu_init_ok =
        MPU6050_Init();


    /*
     * Kalman initialization.
     */
    Kalman1D_Init(
        &full_nav_gyro_filter,
        0.01f,
        0.10f,
        0.0f,
        1.0f);


    /*
     * ==========================================================
     * RESET DEBUG DATA
     * ==========================================================
     */

    debug_fullnav_state =
        FULL_NAV_STARTING;


    debug_fullnav_done =
        0U;


    debug_fullnav_start_station =
        2U;


    debug_fullnav_destination_station =
        3U;


    debug_fullnav_astar_ok =
        0U;


    debug_fullnav_path_length =
        0U;


    debug_fullnav_path_index =
        0U;


    debug_fullnav_target_node =
        (uint8_t)MAP_NODE_INVALID;


    debug_fullnav_ir_pattern =
        0U;


    debug_fullnav_ticks =
        0U;


    debug_fullnav_elapsed_ms =
        0U;


    debug_fullnav_start_delay_ms =
        0U;


    debug_fullnav_destination_confirm =
        0U;


    debug_fullnav_encoder_warning =
        0U;


    debug_fullnav_gyro_bias_dps =
        0.0f;


    debug_fullnav_gyro_cal_samples =
        0U;


    FullNav_ClearDebugPath();


    /*
     * ==========================================================
     * A*
     *
     * START = S2
     * DEST  = S3
     * ==========================================================
     */

    debug_fullnav_astar_ok =
        AStar_FindPath(
            MAP_S2,
            MAP_S3,
            &full_nav_path);


    /*
     * Route failure really must stop navigation.
     */
    if (!debug_fullnav_astar_ok)
    {
        FullNav_Stop(
            FULL_NAV_ASTAR_FAILED);


        while (1)
        {
            Motor_Stop();
        }
    }


    /*
     * Copy route into debug array.
     */
    FullNav_CopyDebugPath();


    /*
     * ==========================================================
     * TWO SECOND STARTUP
     * ==========================================================
     */

    uint32_t start_ticks =
        0U;


    float gyro_sum =
        0.0f;


    while (start_ticks <
           FULL_NAV_START_DELAY_TICKS)
    {
        /*
         * Robot must remain stationary during
         * gyro bias measurement.
         */
        Motor_Stop();


        if (!TIMER_ControlPeriodElapsed())
        {
            continue;
        }


        start_ticks++;


        debug_fullnav_start_delay_ms =
            start_ticks * 10U;


        /*
         * Collect optional stationary gyro data.
         */
        if (debug_fullnav_mpu_init_ok &&
            (debug_fullnav_gyro_cal_samples <
             FULL_NAV_GYRO_CAL_SAMPLES))
        {
            if (MPU6050_Read(
                    &full_nav_mpu))
            {
                gyro_sum +=
                    full_nav_mpu.gyro_z_dps;


                debug_fullnav_gyro_cal_samples++;
            }
        }
    }


    /*
     * Calculate gyro zero-rate bias.
     */
    if (debug_fullnav_gyro_cal_samples > 0U)
    {
        debug_fullnav_gyro_bias_dps =
            gyro_sum /
            (float)
            debug_fullnav_gyro_cal_samples;
    }


    /*
     * Start movement measurements from zero.
     */
    Encoder_Reset();


    Odometry_Init();


    /*
     * ==========================================================
     * START ROUTE
     * ==========================================================
     */

    Navigation_SetPath(
        &full_nav_path);


    debug_fullnav_state =
        FULL_NAV_RUNNING;


    /*
     * ==========================================================
     * MAIN LOOP
     * ==========================================================
     */

    while (debug_fullnav_state ==
           FULL_NAV_RUNNING)
    {
        /*
         * 10 ms control loop.
         */
        if (!TIMER_ControlPeriodElapsed())
        {
            continue;
        }


        /*
         * Update elapsed time.
         */
        debug_fullnav_ticks++;


        debug_fullnav_elapsed_ms =
            debug_fullnav_ticks * 10U;


        /*
         * Monitoring:
         *
         * encoder + MPU + Kalman + odometry.
         */
        FullNav_UpdateMonitoring();


        /*
         * Actual physical steering.
         */
        Navigation_Update();


        /*
         * Read IR for debugger.
         */
        debug_fullnav_ir_pattern =
            IRArray_ReadPattern();


        /*
         * Current graph progress.
         */
        debug_fullnav_path_index =
            Navigation_GetPathIndex();


        /*
         * Current target node.
         */
        if (debug_fullnav_path_index <
            full_nav_path.length)
        {
            debug_fullnav_target_node =
                (uint8_t)
                full_nav_path.nodes[
                    debug_fullnav_path_index];
        }
        else
        {
            debug_fullnav_target_node =
                (uint8_t)MAP_NODE_INVALID;
        }


        /*
         * ======================================================
         * TEMPORARY DESTINATION DETECTION
         * ======================================================
         *
         * RFID currently unavailable.
         *
         * At final route node we temporarily
         * use a confirmed 111 region.
         */

        if (debug_fullnav_path_index ==
            (uint8_t)
            (full_nav_path.length - 1U))
        {
            if (debug_fullnav_ir_pattern ==
                0x07U)
            {
                if (debug_fullnav_destination_confirm <
                    FULL_NAV_DEST_CONFIRM)
                {
                    debug_fullnav_destination_confirm++;
                }
            }
            else
            {
                debug_fullnav_destination_confirm =
                    0U;
            }


            if (debug_fullnav_destination_confirm >=
                FULL_NAV_DEST_CONFIRM)
            {
                FullNav_Stop(
                    FULL_NAV_DESTINATION);


                break;
            }
        }
        else
        {
            debug_fullnav_destination_confirm =
                0U;
        }


        /*
         * ======================================================
         * EMERGENCY TIMEOUT
         * ======================================================
         */

        if (debug_fullnav_ticks >=
            FULL_NAV_TIMEOUT_TICKS)
        {
            FullNav_Stop(
                FULL_NAV_TIMEOUT);


            break;
        }
    }


    /*
     * Safe final state.
     */
    while (1)
    {
        Motor_Stop();
    }
}
