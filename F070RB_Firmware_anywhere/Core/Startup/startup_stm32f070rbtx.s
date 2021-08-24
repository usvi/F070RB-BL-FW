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

  .section .text.Reset_Handler
  .weak Reset_Handler
  .type Reset_Handler, %function
Reset_Handler:
  ldr   r0, =_estack
  msr   msp, r0          /* set stack pointer */
  msr   psp, r0          /* set stack pointer */

	// Store r6 passed by bootloader as gu32FirmwareOffset (earlier: r12)
	ldr r2, =gu32FirmwareOffset
	str r6, [r2]
	// Store r5 passed by bootloader as gu32FirmwareAbsPosition (earlier r11)
	ldr r2, =gu32FirmwareAbsPosition
	str r5, [r2]
	movs r2, #0 // Cleanup
	movs r5, #0
	movs r6, #0







GotPatchLoopInit:
	ldr r6, =gu32FirmwareOffset // Get firmware offset variable address
	ldr r6, [r6]
	movs r0, #0 // Loop variable
GotPatchLoopCond:
	ldr r1, = _got_start_ram
	ldr r2, = _got_end_ram
	subs r2, r2, r1 // How many bytes is the lenght
	cmp r0, r2 // Check if loop is at end
	beq GotPatchEnd // Jump to end if compare equal
GotPatchLoopBody:
	movs r1, r0 // Copy original loop counter value to r1
	adds r0, r0, #4 // Increase original loop counter r0
	ldr r2, = _got_start_ram // Load got ram start
	ldr r3, = _ram_start // Load actual ram start
	subs r2, r2, r3 // r2 now has plain got offset from where ever
	ldr r3, = _flash_start // Start to assemble flash position
	adds r3, r3, r6 // Add firmware offset, which is still at r6
	adds r3, r3, r2 // Add plain offset
	adds r3, r3, r1 // Add loop offset to reading from flash
	ldr r3, [r3] // Load actual table data from flash
	ldr r4, =_ram_start // Assemble limit to check if over start of ram, in which case don't modify (it is ram or a peripheral)
	cmp r3, r4 // Compare address from got and start of ram
	bhs GotStoreTableAddressToRam // If address higher or same (hs) than start of ram, branch to copy got address as is
	ldr r4, =_flash_end // Assemble limit to check if over end of flash, in which case something is just wrong, so branch to store and hope for the best
	cmp r3, r4 // Compare address from got and end of flash
	bhs GotStoreTableAddressToRam // If address address higher or same (hs) than end of flash, branch to store got table address data and hope for the best
	ldr r4, =_flash_start // Assemble limit to check if under start of flash, in which case something is just wrong, so branch to store and hope for the best
	cmp r3, r4 // Compare address from got and start of flash
	blo GotStoreTableAddressToRam // If address address lower (lo) than start of flash, branch to store got table address data and hope for the best
	adds r3, r3, r6 // Finally a position in flash. Add the offset.
GotStoreTableAddressToRam:
	ldr r4, =_ram_start// Start getting address in ram where to put the table address value
	adds r4, r4, r2 // Add plain offset of got
	adds r4, r4, r1 // Add the original loop counter (is: 0, 4, 8, 12, ...)
	str r3, [r4] // Add the table address to ram
	b GotPatchLoopCond // And go to check the loop
GotPatchEnd:
	ldr r0, =_got_start_ram
	mov r9, r0 // Stupid trick to put global offset table location to r9
	movs r0, 0 // Cleaning up the rest, just in case
	movs r1, 0
	movs r2, 0
	movs r3, 0
	movs r4, 0
	movs r5, 0
	movs r6, 0
	movs r7, 0





















/* Copy the data segment initializers from flash to SRAM */
  movs	r1, #0
  b	LoopCopyDataInit

CopyDataInit:
	ldr r7, =gu32FirmwareOffset
	ldr r7, [r7]
	ldr	r3, =_sidata
	adds r3, r3, r7
	ldr	r3, [r3, r1]
	str	r3, [r0, r1]
	adds	r1, r1, #4

LoopCopyDataInit:
	ldr	r0, =_sdata
	ldr	r3, =_edata
	adds	r2, r0, r1
	cmp	r2, r3
	bcc	CopyDataInit
	ldr	r2, =_sbss
	adds r2, r2, r7
	b	LoopFillZerobss
/* Zero fill the bss segment. */
FillZerobss:
	movs	r3, #0
	adds r2, r2, #4 // Increment the loop counter already so ww avoid non-ending loops
	ldr r4, =gu32FirmwareOffset // Get firmware offset variable address
	cmp r2, r4 // Compare address to the address we are going to zero
	beq LoopFillZerobss // Jump away if would otherwise zero it
	ldr r4, =gu32FirmwareAbsPosition // Get firmware abs position variable address
	cmp r2, r4 // Compare address to the address we are going to zero
	beq LoopFillZerobss // Jump away if would otherwise zero it
	subs r2, r2, #4 // Remove our own increment which was needed for special cases
	str	r3, [r2]
	adds r2, #4

LoopFillZerobss:
	ldr r7, =gu32FirmwareOffset
	ldr r7, [r7]
	ldr	r3, =_ebss
	cmp	r2, r3
	bcc	FillZerobss





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
	blx r3
	adds r0, r0, #4
	b CallPreinitsLoopCond
CallPreinitsEnd:

	ldr r3, =_init
	adds r3, r7
	ldr r5, =__init_array_start
	adds r5, r7
	ldr r4, =__init_array_end
	adds r4, r7
	blx r3

	// r4, r5 untouched or good, hopefully
CallInitsInit:
	ldr r7, =gu32FirmwareOffset
	ldr r7, [r7]
CallInitsLoopCond:
	cmp r5, r4
	beq CallInitsEnd
CallInitsLoop:
	ldr r3, [r5]
	add r3, r3, r7
	blx r3
	adds r5, r5, #4
	b CallInitsLoopCond
CallInitsEnd:
	movs r0, #0
	movs r1, #0
	movs r2, #0
	movs r3, #0
	movs r4, #0
	movs r5, #0
	movs r6, #0
	movs r7, #0













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

