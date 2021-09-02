/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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


static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void vF070rb_DeInitAndJump(uint32_t u32JumpAddress);

// From https://github.com/viktorvano/STM32-Bootloader/blob/master/STM32F103C8T6_Bootloader/Core/Inc/bootloader.h
typedef void (application_t)(void);

// From https://github.com/viktorvano/STM32-Bootloader/blob/master/STM32F103C8T6_Bootloader/Core/Inc/bootloader.h
typedef struct
{
    uint32_t    stack_addr;     // Stack Pointer
    application_t*  func_p;        // Program Counter
} JumpStruct;

static void vF070rb_DeInitAndJump(uint32_t u32FwAddress)
{
  uint32_t u32VectorAddress = 0;

  uint32_t* pu32FwFlashPointer = (uint32_t*)u32FwAddress;
  uint32_t* pu32FwRamPointer = (uint32_t*)RAM_VECTOR_TABLE_BEGIN;
  uint32_t u32FirmwareOffset = u32FwAddress - FLASH_BOOTLOADER_BEGIN;
  uint32_t u32UnpatchedValue = 0;
  uint32_t u32PatchedValue = 0;

  // Stop compile nags:
  u32UnpatchedValue = u32UnpatchedValue;
  u32PatchedValue = u32PatchedValue;

  // Check if we need to do reset handler relocation. Not 100% accurate because
  // if original binary reset handler gets pushed back beyond "natural" 0x5000 border, this fails
  uint32_t u32UnalteredResetAddress = *(pu32FwFlashPointer + 1);

  // Cannot figure out right now what corner case could be
  if (u32UnalteredResetAddress < FLASH_FIRMWARES_EARLIEST_BEGIN)
  {
    // Detected actual firmware, so copy and patch it.

    // Copy vector table first
    while (pu32FwRamPointer < (uint32_t*)RAM_VECTOR_TABLE_END)
    {
      *(pu32FwRamPointer++) = *(pu32FwFlashPointer++);
    }
    // Reset is in offset 1
    // Example
    // We are given  u32FwAddress = 0x8005000;
    // Firmware binary thinks it is in 0x8000000 (which is actually bootloader start address)
    // Offset is 0x8005000 - 0x8000000 eq u32FwAddress - FLASH_BOOTLOADER_BEGIN

    // Patch vector table...
    pu32FwRamPointer = (uint32_t*)RAM_VECTOR_TABLE_BEGIN;
    pu32FwRamPointer++; // .. but omit first address, it points to ram

    // Actual patching loop
    while (pu32FwRamPointer < (uint32_t*)RAM_VECTOR_TABLE_END)
    {
      if (*pu32FwRamPointer != 0)
      {
        u32UnpatchedValue = *pu32FwRamPointer;
        *pu32FwRamPointer += u32FirmwareOffset;
        u32PatchedValue = *pu32FwRamPointer;
      }
      pu32FwRamPointer++;
    }
    // Firmware patches its own got in assembly upon startup

    // And lets hope for the best
    u32VectorAddress = RAM_VECTOR_TABLE_BEGIN;
  }
  else
  {
    u32VectorAddress = u32FwAddress;
  }
  // Deinitialization and jump parts from
  // https://github.com/viktorvano/STM32-Bootloader/blob/master/STM32F103C8T6_Bootloader/Core/Inc/bootloader.h
  const JumpStruct* pxJumpVector = ((JumpStruct*)(u32VectorAddress));

  __disable_irq();

  HAL_GPIO_DeInit(LD2_GPIO_Port, LD2_Pin);
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
  HAL_RCC_DeInit();
  HAL_DeInit();

  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL = 0;

  // Cortex-M0 has no vector table.
  // SCB->VTOR = u32VectorAddress;
  // But we can remap memory
  // Lets put it to firmware side
  //__HAL_SYSCFG_REMAPMEMORY_SRAM();
  //__DMB();

  // Store firmware offset to r6 (was: r12 but there was some kind of stupid low register requirement)
  asm ("ldr r6, %0;"
      :"=m"(u32FirmwareOffset)
      :
      :);

  // Store firmware actual address to r5 (was: r11 but there was some kind of stupid low register requirement)
  asm ("ldr r5, %0;"
      :"=m"(u32FwAddress)
      :
      :);

  // Actual jump
  asm("mov sp, %0; bx %1;" : : "r"(pxJumpVector->stack_addr), "r"(pxJumpVector->func_p));

}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  uint32_t u32LedCounter = 0;
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  // Run high frequency for a brief while, then jump
  for (u32LedCounter = 0; u32LedCounter < 0xA0000; u32LedCounter++)
  {
    if ((u32LedCounter % 0x7FFF) == 0)
    {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }
  }
  // Leave LED off
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  // Deinit and jump
  vF070rb_DeInitAndJump(0x8005000);


  while (1)
  {
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
