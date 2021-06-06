#include "user_run.h"

#include <stdbool.h>

#include "stm32h7xx_hal.h"

_Noreturn void userRun(void) {
    while (true) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
        HAL_Delay(500);
    }
}
