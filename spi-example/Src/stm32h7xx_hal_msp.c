/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : stm32h7xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization 
  *                      and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
extern DMA_HandleTypeDef hdma_spi1_tx;

extern DMA_HandleTypeDef hdma_spi3_rx;

extern DMA_HandleTypeDef hdma_spi4_rx;

extern DMA_HandleTypeDef hdma_spi5_tx;

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void) {
    /* USER CODE BEGIN MspInit 0 */

    /* USER CODE END MspInit 0 */

    __HAL_RCC_SYSCFG_CLK_ENABLE();

    /* System interrupt init*/
    /* PendSV_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);

    /* USER CODE BEGIN MspInit 1 */

    /* USER CODE END MspInit 1 */
}

/**
* @brief SPI MSP Initialization
* This function configures the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (hspi->Instance == SPI1) {
        /* USER CODE BEGIN SPI1_MspInit 0 */

        /* USER CODE END SPI1_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_SPI1_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**SPI1 GPIO Configuration
        PA5     ------> SPI1_SCK
        PA7     ------> SPI1_MOSI
        */
        GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* SPI1 DMA Init */
        /* SPI1_TX Init */
        hdma_spi1_tx.Instance = DMA1_Stream0;
        hdma_spi1_tx.Init.Request = DMA_REQUEST_SPI1_TX;
        hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_spi1_tx.Init.Mode = DMA_NORMAL;
        hdma_spi1_tx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_spi1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(hspi, hdmatx, hdma_spi1_tx);

        /* SPI1 interrupt Init */
        HAL_NVIC_SetPriority(SPI1_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(SPI1_IRQn);
        /* USER CODE BEGIN SPI1_MspInit 1 */

        /* USER CODE END SPI1_MspInit 1 */
    } else if (hspi->Instance == SPI3) {
        /* USER CODE BEGIN SPI3_MspInit 0 */

        /* USER CODE END SPI3_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_SPI3_CLK_ENABLE();

        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        /**SPI3 GPIO Configuration
        PB2     ------> SPI3_MOSI
        PC10     ------> SPI3_SCK
        */
        GPIO_InitStruct.Pin = GPIO_PIN_2;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
        GPIO_InitStruct.Alternate = GPIO_AF7_SPI3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
        GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /* SPI3 DMA Init */
        /* SPI3_RX Init */
        hdma_spi3_rx.Instance = DMA1_Stream1;
        hdma_spi3_rx.Init.Request = DMA_REQUEST_SPI3_RX;
        hdma_spi3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_spi3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_spi3_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_spi3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_spi3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_spi3_rx.Init.Mode = DMA_NORMAL;
        hdma_spi3_rx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_spi3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_spi3_rx) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(hspi, hdmarx, hdma_spi3_rx);

        /* SPI3 interrupt Init */
        HAL_NVIC_SetPriority(SPI3_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(SPI3_IRQn);
        /* USER CODE BEGIN SPI3_MspInit 1 */

        /* USER CODE END SPI3_MspInit 1 */
    } else if (hspi->Instance == SPI4) {
        /* USER CODE BEGIN SPI4_MspInit 0 */

        /* USER CODE END SPI4_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_SPI4_CLK_ENABLE();

        __HAL_RCC_GPIOE_CLK_ENABLE();
        /**SPI4 GPIO Configuration
        PE2     ------> SPI4_SCK
        PE6     ------> SPI4_MOSI
        */
        GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        /* SPI4 DMA Init */
        /* SPI4_RX Init */
        hdma_spi4_rx.Instance = DMA2_Stream0;
        hdma_spi4_rx.Init.Request = DMA_REQUEST_SPI4_RX;
        hdma_spi4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_spi4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_spi4_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_spi4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_spi4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_spi4_rx.Init.Mode = DMA_NORMAL;
        hdma_spi4_rx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_spi4_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_spi4_rx) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(hspi, hdmarx, hdma_spi4_rx);

        /* SPI4 interrupt Init */
        HAL_NVIC_SetPriority(SPI4_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(SPI4_IRQn);
        /* USER CODE BEGIN SPI4_MspInit 1 */

        /* USER CODE END SPI4_MspInit 1 */
    } else if (hspi->Instance == SPI5) {
        /* USER CODE BEGIN SPI5_MspInit 0 */

        /* USER CODE END SPI5_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_SPI5_CLK_ENABLE();

        __HAL_RCC_GPIOF_CLK_ENABLE();
        /**SPI5 GPIO Configuration
        PF7     ------> SPI5_SCK
        PF9     ------> SPI5_MOSI
        */
        GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI5;
        HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

        /* SPI5 DMA Init */
        /* SPI5_TX Init */
        hdma_spi5_tx.Instance = DMA2_Stream1;
        hdma_spi5_tx.Init.Request = DMA_REQUEST_SPI5_TX;
        hdma_spi5_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_spi5_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_spi5_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_spi5_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_spi5_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_spi5_tx.Init.Mode = DMA_NORMAL;
        hdma_spi5_tx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_spi5_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_spi5_tx) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(hspi, hdmatx, hdma_spi5_tx);

        /* SPI5 interrupt Init */
        HAL_NVIC_SetPriority(SPI5_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(SPI5_IRQn);
        /* USER CODE BEGIN SPI5_MspInit 1 */

        /* USER CODE END SPI5_MspInit 1 */
    }

}

/**
* @brief SPI MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        /* USER CODE BEGIN SPI1_MspDeInit 0 */

        /* USER CODE END SPI1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_SPI1_CLK_DISABLE();

        /**SPI1 GPIO Configuration
        PA5     ------> SPI1_SCK
        PA7     ------> SPI1_MOSI
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5 | GPIO_PIN_7);

        /* SPI1 DMA DeInit */
        HAL_DMA_DeInit(hspi->hdmatx);

        /* SPI1 interrupt DeInit */
        HAL_NVIC_DisableIRQ(SPI1_IRQn);
        /* USER CODE BEGIN SPI1_MspDeInit 1 */

        /* USER CODE END SPI1_MspDeInit 1 */
    } else if (hspi->Instance == SPI3) {
        /* USER CODE BEGIN SPI3_MspDeInit 0 */

        /* USER CODE END SPI3_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_SPI3_CLK_DISABLE();

        /**SPI3 GPIO Configuration
        PB2     ------> SPI3_MOSI
        PC10     ------> SPI3_SCK
        */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_2);

        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10);

        /* SPI3 DMA DeInit */
        HAL_DMA_DeInit(hspi->hdmarx);

        /* SPI3 interrupt DeInit */
        HAL_NVIC_DisableIRQ(SPI3_IRQn);
        /* USER CODE BEGIN SPI3_MspDeInit 1 */

        /* USER CODE END SPI3_MspDeInit 1 */
    } else if (hspi->Instance == SPI4) {
        /* USER CODE BEGIN SPI4_MspDeInit 0 */

        /* USER CODE END SPI4_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_SPI4_CLK_DISABLE();

        /**SPI4 GPIO Configuration
        PE2     ------> SPI4_SCK
        PE6     ------> SPI4_MOSI
        */
        HAL_GPIO_DeInit(GPIOE, GPIO_PIN_2 | GPIO_PIN_6);

        /* SPI4 DMA DeInit */
        HAL_DMA_DeInit(hspi->hdmarx);

        /* SPI4 interrupt DeInit */
        HAL_NVIC_DisableIRQ(SPI4_IRQn);
        /* USER CODE BEGIN SPI4_MspDeInit 1 */

        /* USER CODE END SPI4_MspDeInit 1 */
    } else if (hspi->Instance == SPI5) {
        /* USER CODE BEGIN SPI5_MspDeInit 0 */

        /* USER CODE END SPI5_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_SPI5_CLK_DISABLE();

        /**SPI5 GPIO Configuration
        PF7     ------> SPI5_SCK
        PF9     ------> SPI5_MOSI
        */
        HAL_GPIO_DeInit(GPIOF, GPIO_PIN_7 | GPIO_PIN_9);

        /* SPI5 DMA DeInit */
        HAL_DMA_DeInit(hspi->hdmatx);

        /* SPI5 interrupt DeInit */
        HAL_NVIC_DisableIRQ(SPI5_IRQn);
        /* USER CODE BEGIN SPI5_MspDeInit 1 */

        /* USER CODE END SPI5_MspDeInit 1 */
    }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
