#include "test_motor.h"

#include "motor.h"

void Test_Motor(void)
{
    Motor_Stop();

    /*
     * SAME API USED BY NAVIGATION.
     *
     * Both motors logical forward at 68%.
     */
    Motor_Set(
        68,
        68);

    while (1)
    {
        /*
         * Re-assert exactly what navigation should
         * request when centred on the line.
         */
        Motor_Set(
            68,
            68);
    }
}
