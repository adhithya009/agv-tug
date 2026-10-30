#include "mpu6050.h"

#include <stdint.h>

#include "i2c_cfg.h"


/*
 * ==============================================================
 * REGISTER ADDRESSES
 * ==============================================================
 */

#define MPU_REG_SMPLRT_DIV      0x19U
#define MPU_REG_CONFIG          0x1AU
#define MPU_REG_GYRO_CONFIG     0x1BU
#define MPU_REG_ACCEL_CONFIG    0x1CU

#define MPU_REG_ACCEL_XOUT_H    0x3BU

#define MPU_REG_PWR_MGMT_1      0x6BU
#define MPU_REG_WHO_AM_I        0x75U


/*
 * ==============================================================
 * SENSOR SCALE FACTORS
 * ==============================================================
 *
 * ACCEL_CONFIG = 0x00
 * +/-2 g
 * 16384 LSB/g
 *
 * GYRO_CONFIG = 0x00
 * +/-250 degrees/second
 * 131 LSB/(degree/second)
 */

#define MPU_ACCEL_SCALE         16384.0f
#define MPU_GYRO_SCALE            131.0f


/*
 * ==============================================================
 * RETRY SETTINGS
 * ==============================================================
 *
 * Our hardware showed that the first I2C register transaction
 * can occasionally fail while the following transaction works.
 *
 * Therefore every important transaction gets several attempts.
 */

#define MPU_I2C_RETRIES         5U


/*
 * Store detected WHO_AM_I value.
 *
 * 0x68 -> standard MPU6050
 * 0x70 -> our MPU6500-compatible device / clone
 */
static uint8_t detected_device_id = 0U;


/*
 * ==============================================================
 * RETRY ONE REGISTER READ
 * ==============================================================
 */

static uint8_t MPU_ReadRegRetry(uint8_t reg,
                                uint8_t *value)
{
    uint8_t attempt;

    if (value == 0)
    {
        return 0U;
    }

    for (attempt = 0U;
         attempt < MPU_I2C_RETRIES;
         attempt++)
    {
        if (I2C1_ReadReg(
                MPU6050_ADDRESS,
                reg,
                value) != 0U)
        {
            return 1U;
        }
    }

    return 0U;
}


/*
 * ==============================================================
 * RETRY ONE REGISTER WRITE
 * ==============================================================
 */

static uint8_t MPU_WriteRegRetry(uint8_t reg,
                                 uint8_t value)
{
    uint8_t attempt;

    for (attempt = 0U;
         attempt < MPU_I2C_RETRIES;
         attempt++)
    {
        if (I2C1_WriteReg(
                MPU6050_ADDRESS,
                reg,
                value) != 0U)
        {
            return 1U;
        }
    }

    return 0U;
}


/*
 * ==============================================================
 * WRITE REGISTER AND VERIFY IT
 * ==============================================================
 *
 * This does more than just trust the write result.
 *
 * We:
 *
 * 1. write the register
 * 2. read it back
 * 3. compare the returned value
 */

static uint8_t MPU_WriteAndVerify(uint8_t reg,
                                  uint8_t value)
{
    uint8_t readback = 0U;
    uint8_t attempt;

    for (attempt = 0U;
         attempt < MPU_I2C_RETRIES;
         attempt++)
    {
        if (MPU_WriteRegRetry(
                reg,
                value) == 0U)
        {
            continue;
        }

        if (MPU_ReadRegRetry(
                reg,
                &readback) == 0U)
        {
            continue;
        }

        if (readback == value)
        {
            return 1U;
        }
    }

    return 0U;
}


/*
 * ==============================================================
 * READ WHO_AM_I
 * ==============================================================
 */

uint8_t MPU6050_ReadWhoAmI(uint8_t *who_am_i)
{
    if (who_am_i == 0)
    {
        return 0U;
    }

    return MPU_ReadRegRetry(
        MPU_REG_WHO_AM_I,
        who_am_i);
}


/*
 * ==============================================================
 * INITIALIZE MPU
 * ==============================================================
 */

uint8_t MPU6050_Init(void)
{
    uint8_t who_am_i = 0U;


    /*
     * ----------------------------------------------------------
     * STEP 1:
     * IDENTIFY SENSOR
     * ----------------------------------------------------------
     *
     * Our tested hardware:
     *
     * I2C address = 0x68
     * WHO_AM_I    = 0x70
     */

    if (MPU6050_ReadWhoAmI(
            &who_am_i) == 0U)
    {
        return 0U;
    }


    /*
     * Accept both:
     *
     * 0x68 -> MPU6050
     * 0x70 -> MPU6500-compatible sensor
     */

    if ((who_am_i != 0x68U) &&
        (who_am_i != 0x70U))
    {
        return 0U;
    }


    detected_device_id =
        who_am_i;


    /*
     * ----------------------------------------------------------
     * STEP 2:
     * WAKE SENSOR
     * ----------------------------------------------------------
     *
     * PWR_MGMT_1 = 0x00
     *
     * Clears the sleep bit.
     *
     * Read-back verification also gives the sensor
     * some transaction time after waking.
     */

    if (MPU_WriteAndVerify(
            MPU_REG_PWR_MGMT_1,
            0x00U) == 0U)
    {
        return 0U;
    }


    /*
     * ----------------------------------------------------------
     * STEP 3:
     * DIGITAL LOW-PASS FILTER
     * ----------------------------------------------------------
     *
     * DLPF_CFG = 3.
     */

    if (MPU_WriteAndVerify(
            MPU_REG_CONFIG,
            0x03U) == 0U)
    {
        return 0U;
    }


    /*
     * ----------------------------------------------------------
     * STEP 4:
     * SAMPLE RATE DIVIDER
     * ----------------------------------------------------------
     *
     * Divider = 9.
     *
     * Approximately:
     *
     * 1000 Hz / (1 + 9)
     * = 100 Hz.
     */

    if (MPU_WriteAndVerify(
            MPU_REG_SMPLRT_DIV,
            0x09U) == 0U)
    {
        return 0U;
    }


    /*
     * ----------------------------------------------------------
     * STEP 5:
     * GYROSCOPE RANGE
     * ----------------------------------------------------------
     *
     * 0x00:
     *
     * +/-250 degrees/second.
     */

    if (MPU_WriteAndVerify(
            MPU_REG_GYRO_CONFIG,
            0x00U) == 0U)
    {
        return 0U;
    }


    /*
     * ----------------------------------------------------------
     * STEP 6:
     * ACCELEROMETER RANGE
     * ----------------------------------------------------------
     *
     * 0x00:
     *
     * +/-2 g.
     */

    if (MPU_WriteAndVerify(
            MPU_REG_ACCEL_CONFIG,
            0x00U) == 0U)
    {
        return 0U;
    }


    /*
     * Initialization succeeded.
     */

    return 1U;
}


/*
 * ==============================================================
 * READ SENSOR DATA
 * ==============================================================
 */

uint8_t MPU6050_Read(MPU6050Data *data)
{
    uint8_t buffer[14];
    uint8_t attempt;
    uint8_t read_ok = 0U;


    if (data == 0)
    {
        return 0U;
    }


    /*
     * ----------------------------------------------------------
     * READ 14 CONSECUTIVE BYTES
     * ----------------------------------------------------------
     *
     * Start register:
     *
     * 0x3B = ACCEL_XOUT_H
     *
     * Layout:
     *
     * AX H/L
     * AY H/L
     * AZ H/L
     *
     * TEMP H/L
     *
     * GX H/L
     * GY H/L
     * GZ H/L
     */

    for (attempt = 0U;
         attempt < MPU_I2C_RETRIES;
         attempt++)
    {
        read_ok =
            I2C1_ReadRegs(
                MPU6050_ADDRESS,
                MPU_REG_ACCEL_XOUT_H,
                buffer,
                14U);

        if (read_ok != 0U)
        {
            break;
        }
    }


    if (read_ok == 0U)
    {
        return 0U;
    }


    /*
     * ----------------------------------------------------------
     * COMBINE HIGH AND LOW BYTES
     * ----------------------------------------------------------
     */


    data->accel_x_raw =
        (int16_t)(
            ((uint16_t)buffer[0] << 8U) |
            buffer[1]);


    data->accel_y_raw =
        (int16_t)(
            ((uint16_t)buffer[2] << 8U) |
            buffer[3]);


    data->accel_z_raw =
        (int16_t)(
            ((uint16_t)buffer[4] << 8U) |
            buffer[5]);


    data->temperature_raw =
        (int16_t)(
            ((uint16_t)buffer[6] << 8U) |
            buffer[7]);


    data->gyro_x_raw =
        (int16_t)(
            ((uint16_t)buffer[8] << 8U) |
            buffer[9]);


    data->gyro_y_raw =
        (int16_t)(
            ((uint16_t)buffer[10] << 8U) |
            buffer[11]);


    data->gyro_z_raw =
        (int16_t)(
            ((uint16_t)buffer[12] << 8U) |
            buffer[13]);


    /*
     * ----------------------------------------------------------
     * ACCELEROMETER CONVERSION
     * ----------------------------------------------------------
     */

    data->accel_x_g =
        (float)data->accel_x_raw /
        MPU_ACCEL_SCALE;


    data->accel_y_g =
        (float)data->accel_y_raw /
        MPU_ACCEL_SCALE;


    data->accel_z_g =
        (float)data->accel_z_raw /
        MPU_ACCEL_SCALE;


    /*
     * ----------------------------------------------------------
     * GYROSCOPE CONVERSION
     * ----------------------------------------------------------
     */

    data->gyro_x_dps =
        (float)data->gyro_x_raw /
        MPU_GYRO_SCALE;


    data->gyro_y_dps =
        (float)data->gyro_y_raw /
        MPU_GYRO_SCALE;


    data->gyro_z_dps =
        (float)data->gyro_z_raw /
        MPU_GYRO_SCALE;


    /*
     * ----------------------------------------------------------
     * TEMPERATURE CONVERSION
     * ----------------------------------------------------------
     *
     * Our hardware reports WHO_AM_I = 0x70,
     * so use MPU6500-compatible conversion.
     */

    if (detected_device_id == 0x70U)
    {
        data->temperature_c =
            ((float)data->temperature_raw /
             333.87f) +
            21.0f;
    }
    else
    {
        data->temperature_c =
            ((float)data->temperature_raw /
             340.0f) +
            36.53f;
    }


    return 1U;
}
