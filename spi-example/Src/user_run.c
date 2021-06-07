#include "user_run.h"

#include <stdbool.h>
#include <string.h>

#include "stm32h7xx_hal.h"
#include "core_cm7.h"

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;

extern SPI_HandleTypeDef hspi4;
extern SPI_HandleTypeDef hspi5;
#define BUFFER_SIZE (40*1024)

static uint16_t buffer[BUFFER_SIZE] __attribute__ ((aligned (32)));
static uint16_t buffer2[BUFFER_SIZE] __attribute__ ((aligned (32)));
static uint16_t buffer3[BUFFER_SIZE] __attribute__ ((aligned (32)));
static uint16_t buffer4[BUFFER_SIZE] __attribute__ ((aligned (32)));

static uint32_t start = 0;
static uint32_t start2 = 0;

bool flag = true;
bool flag2 = true;

_Noreturn void userRun(void) {
    memset(buffer, 0, BUFFER_SIZE);
    memset(buffer4, 0, BUFFER_SIZE);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET);

    for (int i = 0; i < BUFFER_SIZE; ++i) {
        buffer2[i] = i;
        buffer3[i] = i + 10;
    }
    SCB_CleanDCache();

    HAL_SPI_Receive_DMA(&hspi3, (uint8_t *) buffer, BUFFER_SIZE);
    HAL_SPI_Receive_DMA(&hspi4, (uint8_t *) buffer4, BUFFER_SIZE);

    while (true) {
        if (flag) {
            flag = false;
            HAL_Delay(5);
            start = HAL_GetTick();
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
            HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *) buffer2, BUFFER_SIZE);
        }
        if (flag2) {
            flag2 = false;
            HAL_Delay(5);
            start2 = HAL_GetTick();
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);
            HAL_SPI_Transmit_DMA(&hspi5, (uint8_t *) buffer3, BUFFER_SIZE);
        }

        HAL_Delay(5);
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi == &hspi3) {
        uint32_t ticks = HAL_GetTick() - start;
        SCB_CleanInvalidateDCache_by_Addr((uint32_t *) buffer, BUFFER_SIZE * sizeof(uint16_t));
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
        for (int i = 0; i < BUFFER_SIZE; ++i) {
            if (buffer[i] != buffer2[i]) {
                int iii = 1;
            }
        }
        memset(buffer, 0, BUFFER_SIZE);
        SCB_CleanInvalidateDCache_by_Addr((uint32_t *) buffer, BUFFER_SIZE * sizeof(uint16_t));
    } else if (hspi == &hspi4) {
        uint32_t ticks = HAL_GetTick() - start2;
        SCB_CleanInvalidateDCache_by_Addr((uint32_t *) buffer4, BUFFER_SIZE * sizeof(uint16_t));
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
        for (int i = 0; i < BUFFER_SIZE; ++i) {
            if (buffer3[i] != buffer4[i]) {
                int iii = 1;
            }
        }
        memset(buffer4, 0, BUFFER_SIZE);
        SCB_CleanInvalidateDCache_by_Addr((uint32_t *) buffer4, BUFFER_SIZE * sizeof(uint32_t));
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    uint32_t ticks = HAL_GetTick() - start;

    if (hspi == &hspi1) {
//        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
        flag = true;
    } else if (hspi == &hspi5) {
//        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET);
        flag2 = true;
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    int ii = 0;
}
