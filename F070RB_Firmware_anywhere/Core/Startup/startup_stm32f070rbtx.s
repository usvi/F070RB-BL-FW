/**
  ******************************************************************************
  * @file      startup_stm32f070xb.s
  * @author    MCD Application Team
  * @brief     STM32F070xb/STM32F070x8 devices vector table for GCC toolchain.
  *            This module performs:
  *                - Set the initial SP
  *                - Set the initial PC == Reset_Handler,
  *                - Set the vector table entries with the exceptions ISR address
  *                - Branches to main in the C library (which eventually
  *                  calls main()).
  *            After Reset the Cortex-M0 processor is in Thread mode,
  *            priority is Privileged, and the Stack is set to Main.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

  .syntax unified
  .cpu cortex-m0
  .fpu softvfp
  .thumb

.global g_pfnVectors
.global Default_Handler

/* start address for the initialization values of the .data section.
defined in linker script */
.word _sidata
/* start address for the .data section. defined in linker script */
.word _sdata
/* end address for the .data section. defined in linker script */
.word _edata
/* start address for the .bss section. defined in linker script */
.word _sbss
/* end address for the .bss section. defined in linker script */
.word _ebss


.word __flash_begin
.word __ram_vector_table_begin
.word __ram_vector_table_end


  .section .text.Reset_Handler
  .weak Reset_Handler
  .type Reset_Handler, %function
Reset_Handler:
  ldr   r0, =_estack
  mov   sp, r0          /* set stack pointer */



  // Store r10 passed by bootloader as gu32FirmwareAbsPosition, need to use hoop if Cortex-M0
  mov r7, r10
  ldr r2, =gu32FirmwareAbsPosition
  str r7, [r2]

  // Store r11 passed by bootloader as gu32FirmwareOffset, need to use hoop if Cortex-M0
  mov r7, r11
  ldr r2, =gu32FirmwareOffset
  str r7, [r2]

  // Store r12 passed by bootloader as gu32FirmwareAbsOffsetChecksum, need to use hoop if Cortex-M0
  mov r7, r12
  ldr r2, =gu32FirmwareAbsOffsetChecksum
  str r7, [r2]



  // Firmware may be booting as standalone. In that case inspect the checksum
  // and if it does not match, we are most likely running from standalone.
  // Funny thing, Cortex-M0 reset values seem to be like 0xffffffff? Well,
  // checksum in anycase takes care of that correct values are loaded.
  ldr r2, =gu32FirmwareAbsPosition // Load variable address
  ldr r2, [r2] // Load variable data
  ldr r3, =gu32FirmwareOffset // Load variable address
  ldr r3, [r3] // Load variable data
  ldr r4, =gu32FirmwareAbsOffsetChecksum // Load variable address
  ldr r4, [r4] // Load variable data
  movs r1, r2// Calculating the checksum into r1
  eors r1, r1, r3 // r2/gu32FirmwareAbsPosition already there, need only r3/gu32FirmwareOffset
  cmp r1, r4 // Actual compare
  beq StandaloneBootContinue // If match, just do nothing
  // Did not match, so we need to store correct values of gu32FirmwareAbsPosition and gu32FirmwareOffset
  ldr r1, =__flash_begin; // Load variable address
  ldr r2, =gu32FirmwareAbsPosition // Load variable address
  str r1, [r2] // Finally store the new value to ram
  movs r1, #0 // Put zero offset
  ldr r2, =gu32FirmwareOffset // Load firmware offset variable address
  str r1, [r2] // Store zero offset
  // Leave the checksum in memory as it was, even if it was wrong

StandaloneBootContinue:



GotPatchLoopInit:
	ldr r6, =gu32FirmwareOffset // Get firmware offset variable address
	ldr r6, [r6]
	movs r0, #0 // Loop variable

GotPatchLoopCond:
	ldr r1, = __ram_got_begin
	ldr r2, = __ram_got_end
	subs r2, r2, r1 // How many bytes is the lenght
	cmp r0, r2 // Check if loop is at end
	beq GotPatchEnd // Jump to end if compare equal

GotPatchLoopBody:
	movs r1, r0 // Copy original loop counter value to r1
	adds r0, r0, #4 // Increase original loop counter r0
	ldr r2, = __ram_got_begin // Load got ram start
	ldr r3, = __ram_begin // Load actual ram start
	subs r2, r2, r3 // r2 now has plain got offset from where ever
	ldr r3, = __flash_begin // Start to assemble flash position
	adds r3, r3, r6 // Add firmware offset, which is still at r6
	adds r3, r3, r2 // Add plain offset
	adds r3, r3, r1 // Add loop offset to reading from flash
	ldr r3, [r3] // Load actual table data from flash
	ldr r4, =__ram_begin // Assemble limit to check if over start of ram, in which case don't modify (it is ram or a peripheral)
	cmp r3, r4 // Compare address from got and start of ram
	bhs GotStoreTableAddressToRam // If address higher or same (hs) than start of ram, branch to copy got address as is
	ldr r4, =__flash_end // Assemble limit to check if over end of flash, in which case something is just wrong, so branch to store and hope for the best
	cmp r3, r4 // Compare address from got and end of flash
	bhs GotStoreTableAddressToRam // If address address higher or same (hs) than end of flash, branch to store got table address data and hope for the best
	ldr r4, =__flash_begin // Assemble limit to check if under start of flash, in which case something is just wrong, so branch to store and hope for the best
	cmp r3, r4 // Compare address from got and start of flash
	blo GotStoreTableAddressToRam // If address address lower (lo) than start of flash, branch to store got table address data and hope for the best
	adds r3, r3, r6 // Finally a position in flash. Add the offset.

GotStoreTableAddressToRam:
	ldr r4, =__ram_begin// Start getting address in ram where to put the table address value
	adds r4, r4, r2 // Add plain offset of got
	adds r4, r4, r1 // Add the original loop counter (is: 0, 4, 8, 12, ...)
	str r3, [r4] // Add the table address to ram
	b GotPatchLoopCond // And go to check the loop

GotPatchEnd:
	ldr r0, =__ram_got_begin
	mov r9, r0 // Stupid trick to put global offset table location to r9



/* Copy the data segment initializers from flash to SRAM */
  ldr r0, =_sdata
  ldr r1, =_edata
  ldr r2, =_sidata
  ldr r7, =gu32FirmwareOffset // Load firmware offset variable address
  ldr r7, [r7] // Load the actual firmware offset variable data
  adds r2, r2, r7 // Patch the sidata location with offset
  movs r3, #0
  b LoopCopyDataInit

CopyDataInit:
  ldr r4, [r2, r3]
  str r4, [r0, r3]
  adds r3, r3, #4

LoopCopyDataInit:
  adds r4, r0, r3
  cmp r4, r1
  bcc CopyDataInit

/* Zero fill the bss segment. */
  ldr r2, =_sbss
  ldr r4, =_ebss
  movs r3, #0
  b LoopFillZerobss

FillZerobss:
  // Here we need to check that we are not zeroing out addresses or needed symbols

  ldr r6, =gu32FirmwareAbsPosition // Load address of absolute firmware position variable
  cmp r2, r6 // Compare with what we are going to zero
  beq FillZerobssSkip // If we should skip zeroing, jump away

  ldr r6, =gu32FirmwareOffset // Load address of firmware offset variable
  cmp r2, r6 // Compare with what we are going to zero
  beq FillZerobssSkip // If we should skip zeroing, jump away

  ldr r6, =gu32FirmwareAbsOffsetChecksum // Load address of firmware position and offset checksum
  cmp r2, r6 // Compare with what we are going to zero
  beq FillZerobssSkip // If we should skip zeroing, jump away

  ldr r6, =__flash_begin // Load address of flash begin symbol
  cmp r2, r6 // Compare with what we are going to zero
  beq FillZerobssSkip // If we should skip zeroing, jump away

  ldr r6, =__ram_vector_table_begin // Load address of ram vector table begin symbo
  cmp r2, r6 // Compare with what we are going to zero
  beq FillZerobssSkip // If we should skip zeroing, jump away

  ldr r6, =__ram_vector_table_end // Load address of ram vector table end symbol
  cmp r2, r6 // Compare with what we are going to zero
  beq FillZerobssSkip // If we should skip zeroing, jump away

  str  r3, [r2] // If not escaped yet, make the store

FillZerobssSkip:
  adds r2, r2, #4

LoopFillZerobss:
  cmp r2, r4
  bcc FillZerobss



/* Call the clock system intitialization function.*/
  bl  SystemInit



// Make our own __libc_init_array
CallPreinitsInit:
	ldr r7, =gu32FirmwareOffset
	ldr r7, [r7]
	ldr r0, =__preinit_array_start
	adds r0, r7
	ldr r1, =__preinit_array_end
	adds r1, r7

CallPreinitsLoopCond:
	cmp r0, r1
	beq CallPreinitsEnd// If same, it is at end, go away

CallPreinitsLoop:
	ldr r5, =__init_array_start
	ldr r4, =__init_array_end // Yes, order is funny to say the least
	ldr r3, [r0]
	push {r0, r1, r2, r3, r4, r5, r6, r7} // Save context because calling externals
	blx r3
	pop {r0, r1, r2, r3, r4, r5, r6, r7} // Retrieve context
	adds r0, r0, #4
	b CallPreinitsLoopCond

CallPreinitsEnd:
	ldr r3, =_init
	adds r3, r7
	ldr r5, =__init_array_start
	adds r5, r7
	ldr r4, =__init_array_end
	adds r4, r7
  push {r0, r1, r2, r3, r4, r5, r6, r7} // Save context because calling externals
  blx r3
  pop {r0, r1, r2, r3, r4, r5, r6, r7} // Retrieve context

CallInitsInit:
	ldr r7, =gu32FirmwareOffset
	ldr r7, [r7]

CallInitsLoopCond:
	cmp r5, r4
	beq CallInitsEnd

CallInitsLoop:
	ldr r3, [r5]
	add r3, r3, r7
  push {r0, r1, r2, r3, r4, r5, r6, r7} // Save context because calling externals
  blx r3
  pop {r0, r1, r2, r3, r4, r5, r6, r7} // Retrieve context
	adds r5, r5, #4
	b CallInitsLoopCond

CallInitsEnd:



/* Call the application's entry point.*/
  bl main



LoopForever:
  b LoopForever


.size Reset_Handler, .-Reset_Handler

/**
 * @brief  This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 *
 * @param  None
 * @retval : None
*/
    .section .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b Infinite_Loop
  .size Default_Handler, .-Default_Handler
/******************************************************************************
*
* The minimal vector table for a Cortex M0.  Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*
******************************************************************************/
   .section .isr_vector,"a",%progbits
  .type g_pfnVectors, %object
  .size g_pfnVectors, .-g_pfnVectors


g_pfnVectors:
  .word  _estack
  .word  Reset_Handler
  .word  NMI_Handler
  .word  HardFault_Handler
  .word  0
  .word  0
  .word  0
  .word  0
  .word  0
  .word  0
  .word  0
  .word  SVC_Handler
  .word  0
  .word  0
  .word  PendSV_Handler
  .word  SysTick_Handler
  .word  WWDG_IRQHandler                   /* Window WatchDog              */
  .word  0                                 /* Reserved                     */
  .word  RTC_IRQHandler                    /* RTC through the EXTI line    */
  .word  FLASH_IRQHandler                  /* FLASH                        */
  .word  RCC_IRQHandler                    /* RCC                          */
  .word  EXTI0_1_IRQHandler                /* EXTI Line 0 and 1            */
  .word  EXTI2_3_IRQHandler                /* EXTI Line 2 and 3            */
  .word  EXTI4_15_IRQHandler               /* EXTI Line 4 to 15            */
  .word  0                                 /* Reserved                     */
  .word  DMA1_Channel1_IRQHandler          /* DMA1 Channel 1               */
  .word  DMA1_Channel2_3_IRQHandler        /* DMA1 Channel 2 and Channel 3 */
  .word  DMA1_Channel4_5_IRQHandler        /* DMA1 Channel 4 and Channel 5 */
  .word  ADC1_IRQHandler                   /* ADC1                         */
  .word  TIM1_BRK_UP_TRG_COM_IRQHandler    /* TIM1 Break, Update, Trigger and Commutation */
  .word  TIM1_CC_IRQHandler                /* TIM1 Capture Compare         */
  .word  0                                 /* Reserved                     */
  .word  TIM3_IRQHandler                   /* TIM3                         */
  .word  TIM6_IRQHandler                   /* TIM6                         */
  .word  TIM7_IRQHandler                   /* TIM7                         */
  .word  TIM14_IRQHandler                  /* TIM14                        */
  .word  TIM15_IRQHandler                  /* TIM15                        */
  .word  TIM16_IRQHandler                  /* TIM16                        */
  .word  TIM17_IRQHandler                  /* TIM17                        */
  .word  I2C1_IRQHandler                   /* I2C1                         */
  .word  I2C2_IRQHandler                   /* I2C2                         */
  .word  SPI1_IRQHandler                   /* SPI1                         */
  .word  SPI2_IRQHandler                   /* SPI2                         */
  .word  USART1_IRQHandler                 /* USART1                       */
  .word  USART2_IRQHandler                 /* USART2                       */
  .word  USART3_4_IRQHandler               /* USART3 and USART4            */
  .word  0                                 /* Reserved                     */
  .word  USB_IRQHandler                    /* USB                          */

/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/

  .weak      NMI_Handler
  .thumb_set NMI_Handler,Default_Handler

  .weak      HardFault_Handler
  .thumb_set HardFault_Handler,Default_Handler

  .weak      SVC_Handler
  .thumb_set SVC_Handler,Default_Handler

  .weak      PendSV_Handler
  .thumb_set PendSV_Handler,Default_Handler

  .weak      SysTick_Handler
  .thumb_set SysTick_Handler,Default_Handler

  .weak      WWDG_IRQHandler
  .thumb_set WWDG_IRQHandler,Default_Handler

  .weak      RTC_IRQHandler
  .thumb_set RTC_IRQHandler,Default_Handler

  .weak      FLASH_IRQHandler
  .thumb_set FLASH_IRQHandler,Default_Handler

  .weak      RCC_IRQHandler
  .thumb_set RCC_IRQHandler,Default_Handler

  .weak      EXTI0_1_IRQHandler
  .thumb_set EXTI0_1_IRQHandler,Default_Handler

  .weak      EXTI2_3_IRQHandler
  .thumb_set EXTI2_3_IRQHandler,Default_Handler

  .weak      EXTI4_15_IRQHandler
  .thumb_set EXTI4_15_IRQHandler,Default_Handler

  .weak      DMA1_Channel1_IRQHandler
  .thumb_set DMA1_Channel1_IRQHandler,Default_Handler

  .weak      DMA1_Channel2_3_IRQHandler
  .thumb_set DMA1_Channel2_3_IRQHandler,Default_Handler

  .weak      DMA1_Channel4_5_IRQHandler
  .thumb_set DMA1_Channel4_5_IRQHandler,Default_Handler

  .weak      ADC1_IRQHandler
  .thumb_set ADC1_IRQHandler,Default_Handler

  .weak      TIM1_BRK_UP_TRG_COM_IRQHandler
  .thumb_set TIM1_BRK_UP_TRG_COM_IRQHandler,Default_Handler

  .weak      TIM1_CC_IRQHandler
  .thumb_set TIM1_CC_IRQHandler,Default_Handler

  .weak      TIM3_IRQHandler
  .thumb_set TIM3_IRQHandler,Default_Handler

  .weak      TIM6_IRQHandler
  .thumb_set TIM6_IRQHandler,Default_Handler

  .weak      TIM7_IRQHandler
  .thumb_set TIM7_IRQHandler,Default_Handler

  .weak      TIM14_IRQHandler
  .thumb_set TIM14_IRQHandler,Default_Handler

  .weak      TIM15_IRQHandler
  .thumb_set TIM15_IRQHandler,Default_Handler

  .weak      TIM16_IRQHandler
  .thumb_set TIM16_IRQHandler,Default_Handler

  .weak      TIM17_IRQHandler
  .thumb_set TIM17_IRQHandler,Default_Handler

  .weak      I2C1_IRQHandler
  .thumb_set I2C1_IRQHandler,Default_Handler

  .weak      I2C2_IRQHandler
  .thumb_set I2C2_IRQHandler,Default_Handler

  .weak      SPI1_IRQHandler
  .thumb_set SPI1_IRQHandler,Default_Handler

  .weak      SPI2_IRQHandler
  .thumb_set SPI2_IRQHandler,Default_Handler

  .weak      USART1_IRQHandler
  .thumb_set USART1_IRQHandler,Default_Handler

  .weak      USART2_IRQHandler
  .thumb_set USART2_IRQHandler,Default_Handler

  .weak      USART3_4_IRQHandler
  .thumb_set USART3_4_IRQHandler,Default_Handler

  .weak      USB_IRQHandler
  .thumb_set USB_IRQHandler,Default_Handler

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

