#include "test_mpu6050.h"

#include <stdint.h>

#include "mpu6050.h"
#include "motor.h"
#include "kalman.h"


/*
 * ==============================================================
 * COMMUNICATION STATUS
 * ==============================================================
 */

volatile uint8_t debug_mpu_init_ok = 0U;
volatile uint8_t debug_mpu_who_ok = 0U;
volatile uint8_t debug_mpu_who_am_i = 0U;
volatile uint8_t debug_mpu_read_ok = 0U;


/*
 * ==============================================================
 * RAW ACCELEROMETER VALUES
 * ==============================================================
 */

volatile int16_t debug_ax_raw = 0;
volatile int16_t debug_ay_raw = 0;
volatile int16_t debug_az_raw = 0;


/*
 * ==============================================================
 * RAW TEMPERATURE VALUE
 * ==============================================================
 */

volatile int16_t debug_temp_raw = 0;


/*
 * ==============================================================
 * RAW GYROSCOPE VALUES
 * ==============================================================
 */

volatile int16_t debug_gx_raw = 0;
volatile int16_t debug_gy_raw = 0;
volatile int16_t debug_gz_raw = 0;


/*
 * ==============================================================
 * CONVERTED ACCELEROMETER VALUES
 * ==============================================================
 *
 * Unit = g
 */

volatile float debug_ax_g = 0.0f;
volatile float debug_ay_g = 0.0f;
volatile float debug_az_g = 0.0f;


/*
 * ==============================================================
 * CONVERTED GYROSCOPE VALUES
 * ==============================================================
 *
 * Unit = degrees/second
 */

volatile float debug_gx_dps = 0.0f;
volatile float debug_gy_dps = 0.0f;
volatile float debug_gz_dps = 0.0f;


/*
 * ==============================================================
 * KALMAN FILTER VALUES
 * ==============================================================
 *
 * Raw Z-axis gyro measurement.
 */

volatile float debug_gz_raw_dps = 0.0f;


/*
 * Filtered Z-axis gyro measurement.
 */

volatile float debug_gz_filtered_dps = 0.0f;


/*
 * ==============================================================
 * TEMPERATURE
 * ==============================================================
 */

volatile float debug_temperature_c = 0.0f;


/*
 * ==============================================================
 * DRIVER DATA STRUCTURE
 * ==============================================================
 */

static MPU6050Data mpu_data;


/*
 * ==============================================================
 * KALMAN FILTER INSTANCE
 * ==============================================================
 */

static Kalman1D gyro_z_filter;


/*
 * ==============================================================
 * MPU6050 + KALMAN TEST
 * ==============================================================
 */

void Test_MPU6050(void)
{
    uint8_t who_am_i = 0U;


    /*
     * ----------------------------------------------------------
     * SAFETY
     * ----------------------------------------------------------
     *
     * No motor movement is required.
     */

    Motor_Stop();


    /*
     * ----------------------------------------------------------
     * INITIALIZE MPU SENSOR
     * ----------------------------------------------------------
     *
     * Our hardware:
     *
     * I2C address = 0x68
     * WHO_AM_I    = 0x70
     *
     * The updated driver accepts both
     * 0x68 and 0x70 device IDs.
     */

    debug_mpu_init_ok =
        MPU6050_Init();


    /*
     * ----------------------------------------------------------
     * READ WHO_AM_I
     * ----------------------------------------------------------
     */

    debug_mpu_who_ok =
        MPU6050_ReadWhoAmI(
            &who_am_i);


    debug_mpu_who_am_i =
        who_am_i;


    /*
     * ----------------------------------------------------------
     * INITIALIZE KALMAN FILTER
     * ----------------------------------------------------------
     *
     * Q = process noise
     *
     * R = measurement noise
     *
     * Initial estimate = 0
     *
     * Initial covariance = 1
     */

    Kalman1D_Init(
        &gyro_z_filter,
        0.01f,
        0.10f,
        0.0f,
        1.0f);


    /*
     * ----------------------------------------------------------
     * CONTINUOUS SENSOR LOOP
     * ----------------------------------------------------------
     */

    while (1)
    {
        /*
         * Only read the sensor if initialization passed.
         */

        if (debug_mpu_init_ok != 0U)
        {
            debug_mpu_read_ok =
                MPU6050_Read(
                    &mpu_data);
        }
        else
        {
            debug_mpu_read_ok = 0U;
        }


        /*
         * ------------------------------------------------------
         * COPY VALUES ONLY AFTER SUCCESSFUL MPU READ
         * ------------------------------------------------------
         */

        if (debug_mpu_read_ok != 0U)
        {
            /*
             * RAW ACCELEROMETER
             */

            debug_ax_raw =
                mpu_data.accel_x_raw;

            debug_ay_raw =
                mpu_data.accel_y_raw;

            debug_az_raw =
                mpu_data.accel_z_raw;


            /*
             * RAW TEMPERATURE
             */

            debug_temp_raw =
                mpu_data.temperature_raw;


            /*
             * RAW GYROSCOPE
             */

            debug_gx_raw =
                mpu_data.gyro_x_raw;

            debug_gy_raw =
                mpu_data.gyro_y_raw;

            debug_gz_raw =
                mpu_data.gyro_z_raw;


            /*
             * ACCELEROMETER IN g
             */

            debug_ax_g =
                mpu_data.accel_x_g;

            debug_ay_g =
                mpu_data.accel_y_g;

            debug_az_g =
                mpu_data.accel_z_g;


            /*
             * GYROSCOPE IN degrees/second
             */

            debug_gx_dps =
                mpu_data.gyro_x_dps;

            debug_gy_dps =
                mpu_data.gyro_y_dps;

            debug_gz_dps =
                mpu_data.gyro_z_dps;


            /*
             * --------------------------------------------------
             * KALMAN FILTER ON Z-AXIS GYRO
             * --------------------------------------------------
             *
             * Raw gyro Z:
             *
             * mpu_data.gyro_z_dps
             *
             * This is the angular velocity mainly used
             * when the AGV turns left or right.
             */

            debug_gz_raw_dps =
                mpu_data.gyro_z_dps;


            debug_gz_filtered_dps =
                Kalman1D_Update(
                    &gyro_z_filter,
                    debug_gz_raw_dps);


            /*
             * TEMPERATURE
             */

            debug_temperature_c =
                mpu_data.temperature_c;
        }


        /*
         * Keep motors stopped during the entire test.
         */

        Motor_Stop();
    }
}
