#include "user_run.h"

#include <stdbool.h>
#include <string.h>

#include "stm32h7xx_hal.h"
#include "core_cm7.h"

#define BUFFER_SIZE (40*1024)

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;

extern SPI_HandleTypeDef hspi4;
extern SPI_HandleTypeDef hspi5;

static uint16_t line1RxBuffer[BUFFER_SIZE] __attribute__ ((aligned (32)));
static uint16_t line1TxBuffer[BUFFER_SIZE] __attribute__ ((aligned (32)));
static uint16_t line2TxBuffer[BUFFER_SIZE] __attribute__ ((aligned (32)));
static uint16_t line2RxBuffer[BUFFER_SIZE] __attribute__ ((aligned (32)));

static uint32_t line1TxStart = 0;
static uint32_t line2TxStart = 0;

bool flag = true;
bool flag2 = true;

static uint64_t line1Sent = 0;
static uint64_t line2Sent = 0;
static uint64_t line1Received = 0;
static uint64_t line2Received = 0;

SPI_HandleTypeDef *line1TX;
SPI_HandleTypeDef *line1RX;
SPI_HandleTypeDef *line2TX;
SPI_HandleTypeDef *line2RX;

static void setTxBusy(int line, int isBusy);

static void setRxBusy(int line, int isBusy);

static int isTxBusy(void);

static int isRxBusy(void);

static void transmissionTick(void);

_Noreturn void userRun(void) {
    memset(line1RxBuffer, 0, BUFFER_SIZE);
    memset(line2RxBuffer, 0, BUFFER_SIZE);

    // line1TX -> line2RX
    // line1RX <- line2TX
    line1TX = &hspi1;
    line2TX = &hspi5;
    line1RX = &hspi4;
    line2RX = &hspi3;

    for (int i = 0; i < BUFFER_SIZE; ++i) {
        line1TxBuffer[i] = i;
        line2TxBuffer[i] = i + 10;
    }
    SCB_CleanDCache();
    setTxBusy(0, 0);
    setTxBusy(1, 0);
    setRxBusy(0, 1);
    setRxBusy(1, 1);

    while (true) {
        transmissionTick();
        HAL_Delay(5);
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi == line1RX) {
        setRxBusy(1, 1);
        uint32_t ticks = HAL_GetTick() - line2TxStart;
        // received data from peripheral to memory, need to invalidate DCache
        SCB_InvalidateDCache_by_Addr((uint32_t *) line1RxBuffer, BUFFER_SIZE * sizeof(uint16_t));
        bool valid = true;
        for (int i = 0; i < BUFFER_SIZE; ++i) {
            if (line1RxBuffer[i] != line2TxBuffer[i]) {
                valid = false;
                break;
            }
        }
        if (valid) {
            ++line1Received;
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
        }
        float rate = (float) line1Received / (float) line2Sent;
        memset(line1RxBuffer, 0, BUFFER_SIZE);
        // wrote data (with memset) to memory which is used by DMA, need to clean DCache
        SCB_CleanDCache_by_Addr((uint32_t *) line1RxBuffer, BUFFER_SIZE * sizeof(uint16_t));

    } else if (hspi == line2RX) {
        setRxBusy(2, 1);
        uint32_t ticks = HAL_GetTick() - line1TxStart;
        // received data from peripheral to memory, need to invalidate DCache
        SCB_InvalidateDCache_by_Addr((uint32_t *) line2RxBuffer, BUFFER_SIZE * sizeof(uint16_t));
        bool valid = true;
        for (int i = 0; i < BUFFER_SIZE; ++i) {
            if (line2RxBuffer[i] != line1TxBuffer[i]) {
                valid = false;
                break;
            }
        }
        if (valid) {
            ++line2Received;
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
        }
        float rate = (float) line2Received / (float) line1Sent;
        memset(line2RxBuffer, 0, BUFFER_SIZE);
        // wrote data (with memset) to memory which is used by DMA, need to clean DCache
        SCB_CleanDCache_by_Addr((uint32_t *) line2RxBuffer, BUFFER_SIZE * sizeof(uint16_t));
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi == line1TX) {
        setTxBusy(1, 0);
    } else if (hspi == line2TX) {
        setTxBusy(2, 0);
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    int ii = 0;
}

static void setTxBusy(int line, int isBusy) {
    if (line == 1) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, isBusy ? GPIO_PIN_RESET : GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, isBusy ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
}

static void setRxBusy(int line, int isBusy) {
    if (line == 1) {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, isBusy ? GPIO_PIN_RESET : GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, isBusy ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
}

static int isTxBusy(void) {
    return HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14) == GPIO_PIN_RESET;
}

static int isRxBusy(void) {
    return HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_1) == GPIO_PIN_RESET;
}

static void transmissionTick(void) {
    bool tx = isTxBusy();
    bool rx = isRxBusy();
    // check if we can begin receiving
    if (line1RX->State == HAL_SPI_STATE_READY && !isTxBusy()) {
        // we are ready to receive and no one is sending - okay to receive
        setRxBusy(1, 0);
        HAL_SPI_Receive_DMA(line1RX, (uint8_t *) line1RxBuffer, BUFFER_SIZE);
    }
    if (line2RX->State == HAL_SPI_STATE_READY && !isTxBusy()) {
        setRxBusy(2, 0);
        HAL_SPI_Receive_DMA(line2RX, (uint8_t *) line2RxBuffer, BUFFER_SIZE);
    }

    // check if we can begin transmitting
    if (line1TX->State == HAL_SPI_STATE_READY && !isRxBusy()) {
        // we are ready to send and other end is ready to receive
        setTxBusy(1, 1);
        line1TxStart = HAL_GetTick();
        ++line1Sent;
        HAL_SPI_Transmit_DMA(line1TX, (uint8_t *) line1TxBuffer, BUFFER_SIZE);
    }
    if (line2TX->State == HAL_SPI_STATE_READY && !isRxBusy()) {
        // we are ready to send and other end is ready to receive
        setTxBusy(2, 1);
        line2TxStart = HAL_GetTick();
        ++line2Sent;
        HAL_SPI_Transmit_DMA(line2TX, (uint8_t *) line2TxBuffer, BUFFER_SIZE);
    }
}
