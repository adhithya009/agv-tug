#include "test_odometry.h"

#include <stdint.h>

#include "encoder.h"
#include "motor.h"
#include "timer_cfg.h"

#include "mpu6050.h"
#include "kalman.h"
#include "odometry.h"


/*
 * ==============================================================
 * CONTROL PERIOD
 * ==============================================================
 *
 * TIM5 control event occurs every 10 ms.
 *
 * 10 ms = 0.010 seconds.
 */

#define ODOMETRY_DT_S    0.010f


/*
 * ==============================================================
 * STATUS VARIABLES
 * ==============================================================
 */

volatile uint8_t debug_odom_mpu_init_ok = 0U;

volatile uint8_t debug_odom_mpu_read_ok = 0U;


/*
 * ==============================================================
 * ENCODER VALUES
 * ==============================================================
 */

volatile int16_t debug_odom_left_delta = 0;

volatile int16_t debug_odom_right_delta = 0;


volatile uint16_t debug_odom_left_raw = 0U;

volatile uint16_t debug_odom_right_raw = 0U;


/*
 * ==============================================================
 * GYRO VALUES
 * ==============================================================
 */

volatile float debug_odom_gz_raw_dps = 0.0f;

volatile float debug_odom_gz_filtered_dps = 0.0f;


/*
 * ==============================================================
 * ODOMETRY RESULTS
 * ==============================================================
 */

volatile float debug_odom_x_m = 0.0f;

volatile float debug_odom_y_m = 0.0f;

volatile float debug_odom_heading_deg = 0.0f;

volatile float debug_odom_encoder_heading_deg = 0.0f;

volatile float debug_odom_total_distance_m = 0.0f;

volatile float debug_odom_left_distance_m = 0.0f;

volatile float debug_odom_right_distance_m = 0.0f;


/*
 * ==============================================================
 * DRIVER OBJECTS
 * ==============================================================
 */

static MPU6050Data mpu_data;

static Kalman1D gyro_z_filter;


/*
 * ==============================================================
 * ODOMETRY TEST
 * ==============================================================
 */

void Test_Odometry(void)
{
    const OdometryState *odom;


    /*
     * ----------------------------------------------------------
     * SAFETY
     * ----------------------------------------------------------
     *
     * This test uses manual wheel movement first.
     */

    Motor_Stop();


    /*
     * Reset wheel counters.
     */

    Encoder_Reset();


    /*
     * Reset robot position.
     */

    Odometry_Init();


    /*
     * Initialize MPU.
     */

    debug_odom_mpu_init_ok =
        MPU6050_Init();


    /*
     * Initialize Kalman filter for gyro Z.
     */

    Kalman1D_Init(
        &gyro_z_filter,
        0.01f,
        0.10f,
        0.0f,
        1.0f);


    /*
     * ==========================================================
     * MAIN ODOMETRY LOOP
     * ==========================================================
     */

    while (1)
    {
        /*
         * Update every 10 ms.
         */

        if (TIMER_ControlPeriodElapsed())
        {
            /*
             * --------------------------------------------------
             * ENCODERS
             * --------------------------------------------------
             */

            Encoder_Update();


            debug_odom_left_raw =
                Encoder_GetLeftRaw();


            debug_odom_right_raw =
                Encoder_GetRightRaw();


            debug_odom_left_delta =
                Encoder_GetLeftDelta();


            debug_odom_right_delta =
                Encoder_GetRightDelta();


            /*
             * --------------------------------------------------
             * MPU
             * --------------------------------------------------
             */

            if (debug_odom_mpu_init_ok != 0U)
            {
                debug_odom_mpu_read_ok =
                    MPU6050_Read(
                        &mpu_data);
            }
            else
            {
                debug_odom_mpu_read_ok = 0U;
            }


            /*
             * --------------------------------------------------
             * KALMAN FILTER
             * --------------------------------------------------
             */

            if (debug_odom_mpu_read_ok != 0U)
            {
                debug_odom_gz_raw_dps =
                    mpu_data.gyro_z_dps;


                debug_odom_gz_filtered_dps =
                    Kalman1D_Update(
                        &gyro_z_filter,
                        debug_odom_gz_raw_dps);
            }


            /*
             * --------------------------------------------------
             * ODOMETRY UPDATE
             * --------------------------------------------------
             *
             * IMPORTANT:
             *
             * Our LM393 sensors are single-channel.
             *
             * Their hardware counters only increase.
             *
             * During this first test, rotate both wheels
             * FORWARD manually.
             */

            Odometry_Update(
                debug_odom_left_delta,
                debug_odom_right_delta,
                debug_odom_gz_filtered_dps,
                ODOMETRY_DT_S);


            /*
             * --------------------------------------------------
             * GET ODOMETRY STATE
             * --------------------------------------------------
             */

            odom =
                Odometry_GetState();


            /*
             * Copy values for CubeIDE.
             */

            debug_odom_x_m =
                odom->x_m;


            debug_odom_y_m =
                odom->y_m;


            debug_odom_heading_deg =
                odom->heading_deg;


            debug_odom_encoder_heading_deg =
                odom->encoder_heading_deg;


            debug_odom_total_distance_m =
                odom->total_distance_m;


            debug_odom_left_distance_m =
                odom->left_distance_m;


            debug_odom_right_distance_m =
                odom->right_distance_m;
        }


        /*
         * Motors remain OFF during first odometry test.
         */

        Motor_Stop();
    }
}
