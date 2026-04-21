/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SCK_ADS1118_Pin|CS_AD1118_Pin|MOSI_ADS1118_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BULE_GPIO_Port, LED_BULE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, AD7616_RD_Pin|AD7616_WR_Pin|AD7616_CHS0_Pin|AD7616_CS_Pin
                          |AD7616_CHS2_Pin|AD7616_CHS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, AD7616_CONV_Pin|AD7616_RST_Pin|AD7616_RNG1_Pin|AD7616_RNG0_Pin
                          |AD7616_S_P_Pin|FSY_Pin|RES_Pin|SCK_Pin
                          |SDA_Pin|SDA_ad_Pin|SCL_Pin|RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, BLK_Pin|FS_Pin|CS_Pin|PS_Pin
                          |DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : KEY_Pin */
  GPIO_InitStruct.Pin = KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SCK_ADS1118_Pin CS_AD1118_Pin MOSI_ADS1118_Pin */
  GPIO_InitStruct.Pin = SCK_ADS1118_Pin|CS_AD1118_Pin|MOSI_ADS1118_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MISO_ADS1118_Pin */
  GPIO_InitStruct.Pin = MISO_ADS1118_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MISO_ADS1118_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

  
  /*Configure GPIO pin : LED_BULE_Pin */
  GPIO_InitStruct.Pin = LED_BULE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_BULE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AD7616_RD_Pin AD7616_WR_Pin AD7616_CHS0_Pin AD7616_CS_Pin
                           AD7616_CHS2_Pin AD7616_CHS1_Pin */
  GPIO_InitStruct.Pin = AD7616_RD_Pin|AD7616_WR_Pin|AD7616_CHS0_Pin|AD7616_CS_Pin
                          |AD7616_CHS2_Pin|AD7616_CHS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AD7616_CONV_Pin AD7616_RST_Pin AD7616_RNG1_Pin AD7616_RNG0_Pin
                           RES_Pin SDA_Pin SCL_Pin */
  GPIO_InitStruct.Pin = AD7616_CONV_Pin|AD7616_RST_Pin|AD7616_RNG1_Pin|AD7616_RNG0_Pin
                          |RES_Pin|SDA_Pin|SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : AD7616_BUSY_Pin */
  GPIO_InitStruct.Pin = AD7616_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(AD7616_BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : AD7616_S_P_Pin */
  GPIO_InitStruct.Pin = AD7616_S_P_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AD7616_S_P_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AD7616_D15_Pin AD7616_D14_Pin AD7616_D13_Pin AD7616_D12_Pin
                           AD7616_D11_Pin AD7616_D10_Pin AD7616_D9_Pin AD7616_D8_Pin */
  GPIO_InitStruct.Pin = AD7616_D15_Pin|AD7616_D14_Pin|AD7616_D13_Pin|AD7616_D12_Pin
                          |AD7616_D11_Pin|AD7616_D10_Pin|AD7616_D9_Pin|AD7616_D8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : AD7616_D7_Pin AD7616_D6_Pin AD7616_D5_Pin AD7616_D4_Pin */
  GPIO_InitStruct.Pin = AD7616_D7_Pin|AD7616_D6_Pin|AD7616_D5_Pin|AD7616_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : AD7616_D3_Pin AD7616_D2_Pin AD7616_D1_Pin AD7616_D0_Pin */
  GPIO_InitStruct.Pin = AD7616_D3_Pin|AD7616_D2_Pin|AD7616_D1_Pin|AD7616_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BLK_Pin CS_Pin DC_Pin */
  GPIO_InitStruct.Pin = BLK_Pin|CS_Pin|DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : FS_Pin PS_Pin */
  GPIO_InitStruct.Pin = FS_Pin|PS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : FSY_Pin SCK_Pin SDA_ad_Pin RST_Pin */
  GPIO_InitStruct.Pin = FSY_Pin|SCK_Pin|SDA_ad_Pin|RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
