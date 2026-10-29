#ifndef I2C_CFG_H
#define I2C_CFG_H

#include <stdint.h>

/*
 * Configure I2C1:
 *
 * PB8 -> I2C1_SCL
 * PB9 -> I2C1_SDA
 *
 * APB1 clock = 42 MHz
 * I2C speed   = 100 kHz
 */
void I2C1_Init(void);


/*
 * Write one byte into one register of an I2C device.
 *
 * dev_addr is the 7-bit address.
 *
 * Returns:
 * 1 -> success
 * 0 -> communication failure
 */
uint8_t I2C1_WriteReg(uint8_t dev_addr,
                      uint8_t reg_addr,
                      uint8_t data);


/*
 * Read one register.
 *
 * Returns:
 * 1 -> success
 * 0 -> communication failure
 */
uint8_t I2C1_ReadReg(uint8_t dev_addr,
                     uint8_t reg_addr,
                     uint8_t *data);


/*
 * Read several consecutive registers.
 *
 * Used by MPU6050 to read accelerometer,
 * temperature and gyroscope in one transaction.
 */
uint8_t I2C1_ReadRegs(uint8_t dev_addr,
                      uint8_t start_reg,
                      uint8_t *buffer,
                      uint16_t length);

#endif
