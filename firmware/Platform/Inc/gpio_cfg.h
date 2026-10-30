#ifndef GPIO_CFG_H
#define GPIO_CFG_H

#include <stdint.h>


/*
 * Configure all GPIO pins used by the AGV.
 */
void GPIO_Init(void);


/*
 * L298N motor direction control.
 *
 * Left motor:
 * IN1 -> PB1
 * IN2 -> PB15
 *
 * Right motor:
 * IN3 -> PB14
 * IN4 -> PB13
 */
void GPIO_MotorDirection(uint8_t in1,
                         uint8_t in2,
                         uint8_t in3,
                         uint8_t in4);


/*
 * NUCLEO onboard LED.
 *
 * PA5.
 */
void GPIO_LED_Set(uint8_t state);


/*
 * MFRC522 chip select.
 *
 * PB10.
 *
 * 0 -> selected
 * 1 -> deselected
 */
void GPIO_RFID_CS(uint8_t state);


/*
 * MFRC522 reset.
 *
 * PA6.
 *
 * 0 -> reset
 * 1 -> normal operation
 */
void GPIO_RFID_RST(uint8_t state);


#endif
