#include "gpio_cfg.h"

#include "stm32f4xx.h"


/* ==============================================================
 * GPIO CONFIGURATION CONSTANTS
 * ============================================================== */

#define GPIO_MODE_INPUT       0U
#define GPIO_MODE_OUTPUT      1U
#define GPIO_MODE_AF          2U

#define GPIO_PULL_NONE        0U
#define GPIO_PULL_UP          1U

#define GPIO_SPEED_LOW        0U
#define GPIO_SPEED_MEDIUM     1U
#define GPIO_SPEED_FAST       2U


/* ==============================================================
 * INTERNAL HELPERS
 * ============================================================== */

static void GPIO_SetMode(GPIO_TypeDef *port,
                         uint8_t pin,
                         uint32_t mode)
{
    port->MODER &=
        ~(3UL << (pin * 2U));

    port->MODER |=
        (mode << (pin * 2U));
}


static void GPIO_SetPull(GPIO_TypeDef *port,
                         uint8_t pin,
                         uint32_t pull)
{
    port->PUPDR &=
        ~(3UL << (pin * 2U));

    port->PUPDR |=
        (pull << (pin * 2U));
}


static void GPIO_SetSpeed(GPIO_TypeDef *port,
                          uint8_t pin,
                          uint32_t speed)
{
    port->OSPEEDR &=
        ~(3UL << (pin * 2U));

    port->OSPEEDR |=
        (speed << (pin * 2U));
}


static void GPIO_SetOutputType(GPIO_TypeDef *port,
                               uint8_t pin,
                               uint8_t open_drain)
{
    if (open_drain)
    {
        port->OTYPER |=
            (1UL << pin);
    }
    else
    {
        port->OTYPER &=
            ~(1UL << pin);
    }
}


static void GPIO_SetAF(GPIO_TypeDef *port,
                       uint8_t pin,
                       uint8_t af)
{
    uint8_t index =
        pin / 8U;

    uint8_t shift =
        (pin % 8U) * 4U;

    port->AFR[index] &=
        ~(0xFUL << shift);

    port->AFR[index] |=
        ((uint32_t)af << shift);
}


/* ==============================================================
 * GPIO INITIALIZATION
 * ============================================================== */

void GPIO_Init(void)
{
    /* ==========================================================
     * ENABLE GPIO CLOCKS
     * ========================================================== */

    RCC->AHB1ENR |=
        RCC_AHB1ENR_GPIOAEN |
        RCC_AHB1ENR_GPIOBEN |
        RCC_AHB1ENR_GPIOCEN;

    __DSB();


    /* ==========================================================
     * SAFE STARTUP STATES
     * ========================================================== */

    /*
     * Onboard LED OFF.
     */
    GPIOA->BSRR =
        (1UL << (5U + 16U));


    /*
     * Motor direction pins LOW.
     *
     * PB1  -> LEFT IN1
     * PB15 -> LEFT IN2
     * PB14 -> RIGHT IN3
     * PB13 -> RIGHT IN4
     */
    GPIOB->BSRR =
        (1UL << (1U + 16U))  |
        (1UL << (15U + 16U)) |
        (1UL << (14U + 16U)) |
        (1UL << (13U + 16U));


    /*
     * RFID reset HIGH.
     *
     * New RST pin = PA6.
     */
    GPIOA->BSRR =
        (1UL << 6U);


    /*
     * RFID CS HIGH = deselected.
     */
    GPIOB->BSRR =
        (1UL << 10U);


    /* ==========================================================
     * PA0 / A0
     *
     * LINE IR 1 - LEFTMOST
     * ========================================================== */

    GPIO_SetMode(
        GPIOA,
        0U,
        GPIO_MODE_INPUT);

    GPIO_SetPull(
        GPIOA,
        0U,
        GPIO_PULL_UP);


    /* ==========================================================
     * PA1 / A1
     *
     * LINE IR 2 - LEFT-CENTER
     * ========================================================== */

    GPIO_SetMode(
        GPIOA,
        1U,
        GPIO_MODE_INPUT);

    GPIO_SetPull(
        GPIOA,
        1U,
        GPIO_PULL_UP);


    /* ==========================================================
     * PA2
     *
     * USART2 TX
     * ST-Link virtual COM
     * ========================================================== */

    GPIO_SetMode(
        GPIOA,
        2U,
        GPIO_MODE_AF);

    GPIO_SetAF(
        GPIOA,
        2U,
        7U);

    GPIO_SetPull(
        GPIOA,
        2U,
        GPIO_PULL_NONE);

    GPIO_SetSpeed(
        GPIOA,
        2U,
        GPIO_SPEED_MEDIUM);

    GPIO_SetOutputType(
        GPIOA,
        2U,
        0U);


    /* ==========================================================
     * PA3
     *
     * USART2 RX
     * ========================================================== */

    GPIO_SetMode(
        GPIOA,
        3U,
        GPIO_MODE_AF);

    GPIO_SetAF(
        GPIOA,
        3U,
        7U);

    GPIO_SetPull(
        GPIOA,
        3U,
        GPIO_PULL_NONE);


    /* ==========================================================
     * PA4 / A2
     *
     * LINE IR 3 - CENTER
     * ========================================================== */

    GPIO_SetMode(
        GPIOA,
        4U,
        GPIO_MODE_INPUT);

    GPIO_SetPull(
        GPIOA,
        4U,
        GPIO_PULL_UP);


    /* ==========================================================
     * PA5
     *
     * NUCLEO onboard LED
     * ========================================================== */

    GPIO_SetMode(
        GPIOA,
        5U,
        GPIO_MODE_OUTPUT);

    GPIO_SetPull(
        GPIOA,
        5U,
        GPIO_PULL_NONE);

    GPIO_SetSpeed(
        GPIOA,
        5U,
        GPIO_SPEED_LOW);

    GPIO_SetOutputType(
        GPIOA,
        5U,
        0U);


    /* ==========================================================
     * PA6
     *
     * MFRC522 RFID RESET
     *
     * Moved from PB0.
     * ========================================================== */

    GPIO_SetMode(
        GPIOA,
        6U,
        GPIO_MODE_OUTPUT);

    GPIO_SetPull(
        GPIOA,
        6U,
        GPIO_PULL_NONE);

    GPIO_SetSpeed(
        GPIOA,
        6U,
        GPIO_SPEED_LOW);

    GPIO_SetOutputType(
        GPIOA,
        6U,
        0U);


    /* ==========================================================
     * PA8
     *
     * TIM1 CH1
     * LEFT MOTOR ENA PWM
     * ========================================================== */

    GPIO_SetMode(
        GPIOA,
        8U,
        GPIO_MODE_AF);

    GPIO_SetAF(
        GPIOA,
        8U,
        1U);

    GPIO_SetPull(
        GPIOA,
        8U,
        GPIO_PULL_NONE);

    GPIO_SetSpeed(
        GPIOA,
        8U,
        GPIO_SPEED_FAST);

    GPIO_SetOutputType(
        GPIOA,
        8U,
        0U);


    /* ==========================================================
     * PA9
     *
     * USART1 TX
     *
     * STM32 -> HC-05
     * ========================================================== */

    GPIO_SetMode(
        GPIOA,
        9U,
        GPIO_MODE_AF);

    GPIO_SetAF(
        GPIOA,
        9U,
        7U);

    GPIO_SetPull(
        GPIOA,
        9U,
        GPIO_PULL_NONE);

    GPIO_SetSpeed(
        GPIOA,
        9U,
        GPIO_SPEED_MEDIUM);

    GPIO_SetOutputType(
        GPIOA,
        9U,
        0U);


    /* ==========================================================
     * PA10
     *
     * USART1 RX
     *
     * HC-05 -> STM32
     * ========================================================== */

    GPIO_SetMode(
        GPIOA,
        10U,
        GPIO_MODE_AF);

    GPIO_SetAF(
        GPIOA,
        10U,
        7U);

    GPIO_SetPull(
        GPIOA,
        10U,
        GPIO_PULL_NONE);


    /* ==========================================================
     * PA11
     *
     * TIM1 CH4
     * RIGHT MOTOR ENB PWM
     * ========================================================== */

    GPIO_SetMode(
        GPIOA,
        11U,
        GPIO_MODE_AF);

    GPIO_SetAF(
        GPIOA,
        11U,
        1U);

    GPIO_SetPull(
        GPIOA,
        11U,
        GPIO_PULL_NONE);

    GPIO_SetSpeed(
        GPIOA,
        11U,
        GPIO_SPEED_FAST);

    GPIO_SetOutputType(
        GPIOA,
        11U,
        0U);


    /* ==========================================================
     * PB0 / A3
     *
     * LINE IR 4 - RIGHT-CENTER
     *
     * RFID reset is NO LONGER on this pin.
     * ========================================================== */

    GPIO_SetMode(
        GPIOB,
        0U,
        GPIO_MODE_INPUT);

    GPIO_SetPull(
        GPIOB,
        0U,
        GPIO_PULL_UP);


    /* ==========================================================
     * PB1
     *
     * LEFT MOTOR IN1
     * ========================================================== */

    GPIO_SetMode(
        GPIOB,
        1U,
        GPIO_MODE_OUTPUT);

    GPIO_SetPull(
        GPIOB,
        1U,
        GPIO_PULL_NONE);

    GPIO_SetSpeed(
        GPIOB,
        1U,
        GPIO_SPEED_LOW);

    GPIO_SetOutputType(
        GPIOB,
        1U,
        0U);


    /* ==========================================================
     * PB3
     *
     * SPI1 SCK
     * MFRC522 clock
     * ========================================================== */

    GPIO_SetMode(
        GPIOB,
        3U,
        GPIO_MODE_AF);

    GPIO_SetAF(
        GPIOB,
        3U,
        5U);

    GPIO_SetPull(
        GPIOB,
        3U,
        GPIO_PULL_NONE);

    GPIO_SetSpeed(
        GPIOB,
        3U,
        GPIO_SPEED_FAST);

    GPIO_SetOutputType(
        GPIOB,
        3U,
        0U);


    /* ==========================================================
     * PB4
     *
     * SPI1 MISO
     * MFRC522 -> STM32
     * ========================================================== */

    GPIO_SetMode(
        GPIOB,
        4U,
        GPIO_MODE_AF);

    GPIO_SetAF(
        GPIOB,
        4U,
        5U);

    GPIO_SetPull(
        GPIOB,
        4U,
        GPIO_PULL_NONE);


    /* ==========================================================
     * PB5
     *
     * SPI1 MOSI
     * STM32 -> MFRC522
     * ========================================================== */

    GPIO_SetMode(
        GPIOB,
        5U,
        GPIO_MODE_AF);

    GPIO_SetAF(
        GPIOB,
        5U,
        5U);

    GPIO_SetPull(
        GPIOB,
        5U,
        GPIO_PULL_NONE);

    GPIO_SetSpeed(
        GPIOB,
        5U,
        GPIO_SPEED_FAST);

    GPIO_SetOutputType(
        GPIOB,
        5U,
        0U);


    /* ==========================================================
     * PB6
     *
     * TIM4 CH1
     * RIGHT ENCODER
     * ========================================================== */

    GPIO_SetMode(
        GPIOB,
        6U,
        GPIO_MODE_AF);

    GPIO_SetAF(
        GPIOB,
        6U,
        2U);

    GPIO_SetPull(
        GPIOB,
        6U,
        GPIO_PULL_UP);


    /* ==========================================================
     * PB8
     *
     * I2C1 SCL
     * MPU6050
     * ========================================================== */

    GPIO_SetMode(
        GPIOB,
        8U,
        GPIO_MODE_AF);

    GPIO_SetAF(
        GPIOB,
        8U,
        4U);

    GPIO_SetPull(
        GPIOB,
        8U,
        GPIO_PULL_UP);

    GPIO_SetSpeed(
        GPIOB,
        8U,
        GPIO_SPEED_FAST);

    GPIO_SetOutputType(
        GPIOB,
        8U,
        1U);


    /* ==========================================================
     * PB9
     *
     * I2C1 SDA
     * MPU6050
     * ========================================================== */

    GPIO_SetMode(
        GPIOB,
        9U,
        GPIO_MODE_AF);

    GPIO_SetAF(
        GPIOB,
        9U,
        4U);

    GPIO_SetPull(
        GPIOB,
        9U,
        GPIO_PULL_UP);

    GPIO_SetSpeed(
        GPIOB,
        9U,
        GPIO_SPEED_FAST);

    GPIO_SetOutputType(
        GPIOB,
        9U,
        1U);


    /* ==========================================================
     * PB10
     *
     * MFRC522 CS / SDA / SS
     * ========================================================== */

    GPIO_SetMode(
        GPIOB,
        10U,
        GPIO_MODE_OUTPUT);

    GPIO_SetPull(
        GPIOB,
        10U,
        GPIO_PULL_NONE);

    GPIO_SetSpeed(
        GPIOB,
        10U,
        GPIO_SPEED_FAST);

    GPIO_SetOutputType(
        GPIOB,
        10U,
        0U);


    /* ==========================================================
     * PB13
     *
     * RIGHT MOTOR IN4
     * ========================================================== */

    GPIO_SetMode(
        GPIOB,
        13U,
        GPIO_MODE_OUTPUT);

    GPIO_SetPull(
        GPIOB,
        13U,
        GPIO_PULL_NONE);

    GPIO_SetSpeed(
        GPIOB,
        13U,
        GPIO_SPEED_LOW);

    GPIO_SetOutputType(
        GPIOB,
        13U,
        0U);


    /* ==========================================================
     * PB14
     *
     * RIGHT MOTOR IN3
     * ========================================================== */

    GPIO_SetMode(
        GPIOB,
        14U,
        GPIO_MODE_OUTPUT);

    GPIO_SetPull(
        GPIOB,
        14U,
        GPIO_PULL_NONE);

    GPIO_SetSpeed(
        GPIOB,
        14U,
        GPIO_SPEED_LOW);

    GPIO_SetOutputType(
        GPIOB,
        14U,
        0U);


    /* ==========================================================
     * PB15
     *
     * LEFT MOTOR IN2
     * ========================================================== */

    GPIO_SetMode(
        GPIOB,
        15U,
        GPIO_MODE_OUTPUT);

    GPIO_SetPull(
        GPIOB,
        15U,
        GPIO_PULL_NONE);

    GPIO_SetSpeed(
        GPIOB,
        15U,
        GPIO_SPEED_LOW);

    GPIO_SetOutputType(
        GPIOB,
        15U,
        0U);


    /* ==========================================================
     * PC0 / A5
     *
     * OBSTACLE IR
     * ========================================================== */

    GPIO_SetMode(
        GPIOC,
        0U,
        GPIO_MODE_INPUT);

    GPIO_SetPull(
        GPIOC,
        0U,
        GPIO_PULL_UP);


    /* ==========================================================
     * PC1 / A4
     *
     * LINE IR 5 - RIGHTMOST
     * ========================================================== */

    GPIO_SetMode(
        GPIOC,
        1U,
        GPIO_MODE_INPUT);

    GPIO_SetPull(
        GPIOC,
        1U,
        GPIO_PULL_UP);


    /* ==========================================================
     * PC6
     *
     * TIM3 CH1
     * LEFT ENCODER
     * ========================================================== */

    GPIO_SetMode(
        GPIOC,
        6U,
        GPIO_MODE_AF);

    GPIO_SetAF(
        GPIOC,
        6U,
        2U);

    GPIO_SetPull(
        GPIOC,
        6U,
        GPIO_PULL_UP);
}


/* ==============================================================
 * MOTOR DIRECTION CONTROL
 * ============================================================== */

void GPIO_MotorDirection(uint8_t in1,
                         uint8_t in2,
                         uint8_t in3,
                         uint8_t in4)
{
    uint32_t bsrr = 0U;


    /*
     * PB1 -> LEFT IN1
     */
    if (in1)
    {
        bsrr |=
            (1UL << 1U);
    }
    else
    {
        bsrr |=
            (1UL << (1U + 16U));
    }


    /*
     * PB15 -> LEFT IN2
     */
    if (in2)
    {
        bsrr |=
            (1UL << 15U);
    }
    else
    {
        bsrr |=
            (1UL << (15U + 16U));
    }


    /*
     * PB14 -> RIGHT IN3
     */
    if (in3)
    {
        bsrr |=
            (1UL << 14U);
    }
    else
    {
        bsrr |=
            (1UL << (14U + 16U));
    }


    /*
     * PB13 -> RIGHT IN4
     */
    if (in4)
    {
        bsrr |=
            (1UL << 13U);
    }
    else
    {
        bsrr |=
            (1UL << (13U + 16U));
    }


    /*
     * Atomic update of all motor direction pins.
     */
    GPIOB->BSRR =
        bsrr;
}


/* ==============================================================
 * ONBOARD LED
 * ============================================================== */

void GPIO_LED_Set(uint8_t state)
{
    if (state)
    {
        GPIOA->BSRR =
            (1UL << 5U);
    }
    else
    {
        GPIOA->BSRR =
            (1UL << (5U + 16U));
    }
}


/* ==============================================================
 * RFID CHIP SELECT
 *
 * PB10
 * ============================================================== */

void GPIO_RFID_CS(uint8_t state)
{
    if (state)
    {
        /*
         * HIGH = deselected.
         */
        GPIOB->BSRR =
            (1UL << 10U);
    }
    else
    {
        /*
         * LOW = selected.
         */
        GPIOB->BSRR =
            (1UL << (10U + 16U));
    }
}


/* ==============================================================
 * RFID RESET
 *
 * NEW PIN = PA6
 * ============================================================== */

void GPIO_RFID_RST(uint8_t state)
{
    if (state)
    {
        /*
         * HIGH = normal operation.
         */
        GPIOA->BSRR =
            (1UL << 6U);
    }
    else
    {
        /*
         * LOW = reset.
         */
        GPIOA->BSRR =
            (1UL << (6U + 16U));
    }
}
