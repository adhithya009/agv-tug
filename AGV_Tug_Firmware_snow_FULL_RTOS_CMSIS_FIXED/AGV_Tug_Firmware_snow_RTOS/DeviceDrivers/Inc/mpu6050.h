#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

/*
 * MPU6050 7-bit I2C address.
 *
 */
#define MPU6050_ADDRESS 0x68U

typedef struct
{
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;

    int16_t temperature_raw;

    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;

    float accel_x_g;
    float accel_y_g;
    float accel_z_g;

    float temperature_c;

    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

} MPU6050Data;


/*
 * Initialize and wake MPU6050.
 *
 * Returns:
 * 1 -> success
 * 0 -> failure
 */
uint8_t MPU6050_Init(void);


/*
 * Read WHO_AM_I register.
 *
 * Expected value for MPU6050 = 0x68.
 */
uint8_t MPU6050_ReadWhoAmI(uint8_t *who_am_i);


/*
 * Read accelerometer, temperature and gyroscope.
 *
 * Returns:
 * 1 -> success
 * 0 -> failure
 */
uint8_t MPU6050_Read(MPU6050Data *data);

#endif
