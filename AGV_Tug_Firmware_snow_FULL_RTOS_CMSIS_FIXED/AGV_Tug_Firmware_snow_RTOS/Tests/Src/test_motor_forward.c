#include "stm32f4xx.h"
#include "gpio_cfg.h"
#include "timer_cfg.h"

void Test_RightWheel_Forward(void)
{
    /* HW-166 / L298N:
       PB14 -> IN3
       PB13 -> IN4
       PA11 -> ENB / TIM1_CH4 */

    GPIO_MotorDirection(
        0U,   /* Left IN1 */
        0U,   /* Left IN2 */
        1U,   /* Right IN3 = HIGH */
        0U);  /* Right IN4 = LOW */

    TIMER_SetLeftPWM(0U);
    TIMER_SetRightPWM(80U);

    while (1)
    {
        /* Keep right wheel running forward */
    }
}
