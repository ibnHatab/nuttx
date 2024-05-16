/****************************************************************************
 * arch/arm/src/j721e/hardware/j721e_memorymap.h
 *
 * Generated from j721e.svd originally provided by
 *   Raspberry Pi (Trading) Ltd.
 *
 * Copyright 2020 (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_J721E_HARDWARE_J721E_MEMORYMAP_H
#define __ARCH_ARM_SRC_J721E_HARDWARE_J721E_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm_internal.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define J721E_FLASH_BASE                0x10000000 /* -0x001fffff: FLASH memory space (2048KB) */
#define J721E_SRAM_BASE                 0x20000000 /* -0x20041fff: SRAM memory space (264KB) */

#define J721E_XIP_CTRL_BASE             0x14000000  /* QSPI flash execute-in-place block */
#define J721E_XIP_SSI_BASE              0x18000000
#define J721E_SYSINFO_BASE              0x40000000
#define J721E_SYSCFG_BASE               0x40004000  /* Register block for various chip control signals */
#define J721E_CLOCKS_BASE               0x40008000
#define J721E_RESETS_BASE               0x4000c000
#define J721E_PSM_BASE                  0x40010000
#define J721E_IO_BANK0_BASE             0x40014000
#define J721E_IO_QSPI_BASE              0x40018000
#define J721E_PADS_BANK0_BASE           0x4001c000
#define J721E_PADS_QSPI_BASE            0x40020000
#define J721E_XOSC_BASE                 0x40024000  /* Controls the crystal oscillator */
#define J721E_PLL_SYS_BASE              0x40028000
#define J721E_PLL_USB_BASE              0x4002c000
#define J721E_BUSCTRL_BASE              0x40030000  /* Register block for busfabric control signals and performance counters */
#define J721E_UART0_BASE                0x40034000
#define J721E_UART1_BASE                0x40038000
#define J721E_UART_BASE(n)              (0x40034000 + (n) * 0x4000)
#define J721E_SPI0_BASE                 0x4003c000
#define J721E_SPI1_BASE                 0x40040000
#define J721E_SPI_BASE(n)               (0x4003c000 + (n) * 0x4000)
#define J721E_I2C0_BASE                 0x40044000  /* DW_apb_i2c address block */
#define J721E_I2C1_BASE                 0x40048000  /* DW_apb_i2c address block */
#define J721E_I2C_BASE(n)               (0x40044000 + (n) * 0x4000)
#define J721E_ADC_BASE                  0x4004c000  /* Control and data interface to SAR ADC */
#define J721E_PWM_BASE                  0x40050000  /* Simple PWM */
#define J721E_TIMER_BASE                0x40054000  /* Controls time and alarms time is a 64 bit value indicating the time in usec since power-on timeh is the top 32 bits of time & timel is the bottom 32 bits to change time write to timelw before timehw to read time read from timelr before timehr An alarm is set by setting alarm_enable and writing to the corresponding alarm register When an alarm is pending, the corresponding alarm_running signal will be high An alarm can be cancelled before it has finished by clearing the alarm_enable When an alarm fires, the corresponding alarm_irq is set and alarm_running is cleared To clear the interrupt write a 1 to the corresponding alarm_irq */
#define J721E_WATCHDOG_BASE             0x40058000
#define J721E_RTC_BASE                  0x4005c000  /* Register block to control RTC */
#define J721E_ROSC_BASE                 0x40060000
#define J721E_VREG_AND_CHIP_RESET_BASE  0x40064000  /* control and status for on-chip voltage regulator and chip level reset subsystem */
#define J721E_TBMAN_BASE                0x4006c000  /* Testbench manager. Allows the programmer to know what platform their software is running on. */
#define J721E_DMA_BASE                  0x50000000  /* DMA with separate read and write masters */
#define J721E_USBCTRL_DPSRAM_BASE       0x50100000  /* USB Dual Port SRAM */
#define J721E_USBCTRL_REGS_BASE         0x50110000  /* USB FS/LS controller device registers */
#define J721E_PIO0_BASE                 0x50200000  /* Programmable IO block */
#define J721E_PIO1_BASE                 0x50300000  /* Programmable IO block */
#define J721E_PIO_BASE(n)               (0x50200000 + (n) * 0x100000)
#define J721E_SIO_BASE                  0xd0000000  /* Single-cycle IO block Provides core-local and inter-core hardware for the two processors, with single-cycle access. */
#define J721E_PPB_BASE                  0xe0000000

#define J721E_ATOMIC_XOR_REG_OFFSET         0x1000
#define J721E_ATOMIC_SET_REG_OFFSET         0x2000
#define J721E_ATOMIC_CLR_REG_OFFSET         0x3000

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

#  define xorbits_reg32(v,a)   putreg32(v, (a) | J721E_ATOMIC_XOR_REG_OFFSET)
#  define setbits_reg32(v,a)   putreg32(v, (a) | J721E_ATOMIC_SET_REG_OFFSET)
#  define clrbits_reg32(v,a)   putreg32(v, (a) | J721E_ATOMIC_CLR_REG_OFFSET)
#  define modbits_reg32(v,m,a) xorbits_reg32((getreg32(a) ^ (v)) & (m), a)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_J721E_HARDWARE_J721E_MEMORYMAP_H */
