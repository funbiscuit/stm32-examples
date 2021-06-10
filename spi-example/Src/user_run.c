#include "user_run.h"
#include "spi_protocol.h"

#include <stdbool.h>
#include <string.h>

#include "stm32h7xx_hal.h"
#include "core_cm7.h"
#include <cmsis_os2.h>

#define BUFFER_SIZE (16*1024)
//#define BUFFER_SIZE (64)

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;

extern SPI_HandleTypeDef hspi4;
extern SPI_HandleTypeDef hspi5;

static uint32_t tx1[BUFFER_SIZE];
static uint32_t tx2[BUFFER_SIZE];

static uint32_t rx1[BUFFER_SIZE];
static uint32_t rx2[BUFFER_SIZE];

static uint32_t ch1start;
static uint32_t ch2start;

static uint64_t ch1received = 0;
static uint64_t ch2received = 0;

static uint32_t ch1offset = 0;
static uint32_t ch2offset = 0;

float speed;

static void onDataCh1(const uint8_t *data, uint32_t size);

static void onDataCh2(const uint8_t *data, uint32_t size);

static void delay(uint32_t ticks);

static void setTx1Busy(bool isBusy);

static void setTx2Busy(bool isBusy);

static void setRx1Busy(bool isBusy);

static void setRx2Busy(bool isBusy);

static bool isTx1Busy(void);

static bool isTx2Busy(void);

static bool isRx1Busy(void);

static bool isRx2Busy(void);

_Noreturn void taskChannel1(void) {
    // hspi1 -> hspi3
    // hspi5 -> hspi4

    // tx1 busy PD14 <-> PE4 rx2 busy
    // rx1 busy PG1  <-> PA4 tx2 busy
    // when tx is busy - it's transmitting
    // when rx is busy - it's receiving
    // busy = low state

    SpiProtocolInit ch1init;

    ch1init.delay = &HAL_Delay;
    ch1init.isTxBusy = &isTx1Busy;
    ch1init.isRxBusy = &isRx1Busy;
    ch1init.onData = &onDataCh1;

    ch1init.tx = &hspi1;
    ch1init.rx = &hspi4;
    ch1init.setTxBusy = &setTx1Busy;
    ch1init.setRxBusy = &setRx1Busy;

    SpiProtocol *ch1 = spiProtocolInitialize(&ch1init, 0);

    ch1start = HAL_GetTick();
    int j = 0;
    while (true) {
        for (int i = 0; i < BUFFER_SIZE; ++i) {
            tx1[i] = j;
            j += 3;
        }
        ch2offset = 0;
        memset(rx2, 0, BUFFER_SIZE * sizeof(uint16_t));

        spiProtocolWrite(ch1, tx1, BUFFER_SIZE * sizeof(uint32_t));

        for (int i = 0; i < BUFFER_SIZE; ++i) {
            if (rx2[i] != tx1[i]) {
                speed *= 1;
            }
        }

        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
        osDelay(1);
    }
}

_Noreturn void taskChannel2(void) {
    // hspi1 -> hspi3
    // hspi5 -> hspi4

    SpiProtocolInit ch2init;

    ch2init.delay = &delay;
    ch2init.isTxBusy = &isTx2Busy;
    ch2init.isRxBusy = &isRx2Busy;
    ch2init.onData = &onDataCh2;

    ch2init.tx = &hspi5;
    ch2init.rx = &hspi3;
    ch2init.setTxBusy = &setTx2Busy;
    ch2init.setRxBusy = &setRx2Busy;


    SpiProtocol *ch2 = spiProtocolInitialize(&ch2init, 1);

    int j = 13;
    ch2start = HAL_GetTick();
    while (true) {
        for (int i = 0; i < BUFFER_SIZE; ++i) {
            tx2[i] = j;
            j += 1;
        }
        ch1offset = 0;
        memset(rx1, 0, BUFFER_SIZE * sizeof(uint16_t));

        spiProtocolWrite(ch2, tx2, BUFFER_SIZE * sizeof(uint32_t));

        for (int i = 0; i < BUFFER_SIZE; ++i) {
            if (rx1[i] != tx2[i]) {
                speed *= 1;
            }
        }
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
        osDelay(1);
    }
}

static void onDataCh1(const uint8_t *data, uint32_t size) {
    memcpy(&rx1[ch1offset], data, size);
    ch1offset += size / sizeof(uint32_t);
    ch1received += size;
    uint32_t ticks = HAL_GetTick() - ch1start;
    speed = ((float) ch1received) / ((float) ticks);
    speed *= 1;
}

static void onDataCh2(const uint8_t *data, uint32_t size) {
    memcpy(&rx2[ch2offset], data, size);
    ch2offset += size / sizeof(uint32_t);
    ch2received += size;
    uint32_t ticks = HAL_GetTick() - ch2start;
    speed = ((float) ch2received) / ((float) ticks);
    speed *= 1;
}

static void delay(uint32_t ticks) {
    osDelay(ticks);
}

static void setTx1Busy(bool isBusy) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, isBusy ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void setTx2Busy(bool isBusy) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, isBusy ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void setRx1Busy(bool isBusy) {
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, isBusy ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void setRx2Busy(bool isBusy) {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, isBusy ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static bool isTx1Busy(void) {
    return HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14) == GPIO_PIN_RESET;
}

static bool isTx2Busy(void) {
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET;
}

static bool isRx1Busy(void) {
    return HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_1) == GPIO_PIN_RESET;
}

static bool isRx2Busy(void) {
    return HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == GPIO_PIN_RESET;
}
