/****************************************************************************
 * arch/arm/src/j721e/hardware/j721e_pads_bank0.h
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

#ifndef __ARCH_ARM_SRC_J721E_HARDWARE_J721E_PADS_BANK0_H
#define __ARCH_ARM_SRC_J721E_HARDWARE_J721E_PADS_BANK0_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/j721e_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define J721E_PADS_BANK0_VOLTAGE_SELECT_OFFSET  0x000000               /* Voltage select. Per bank control */
#define J721E_PADS_BANK0_GPIO_OFFSET(n)         ((n) * 4 + 0x000004)   /* Pad control register */
#define J721E_PADS_BANK0_SWCLK_OFFSET           0x00007c               /* Pad control register */
#define J721E_PADS_BANK0_SWD_OFFSET             0x000080               /* Pad control register */

/* Register definitions *****************************************************/

#define J721E_PADS_BANK0_VOLTAGE_SELECT  (J721E_PADS_BANK0_BASE + J721E_PADS_BANK0_VOLTAGE_SELECT_OFFSET)
#define J721E_PADS_BANK0_GPIO(n)         (J721E_PADS_BANK0_BASE + J721E_PADS_BANK0_GPIO_OFFSET(n))
#define J721E_PADS_BANK0_SWCLK           (J721E_PADS_BANK0_BASE + J721E_PADS_BANK0_SWCLK_OFFSET)
#define J721E_PADS_BANK0_SWD             (J721E_PADS_BANK0_BASE + J721E_PADS_BANK0_SWD_OFFSET)

/* Register bit definitions *************************************************/

#define J721E_PADS_BANK0_VOLTAGE_SELECT_1_8V (1 << 0)  /* Set voltage to 1.8V (DVDD <= 1V8) */

#define J721E_PADS_BANK0_GPIO_OD             (1 << 7)  /* Output disable. Has priority over output enable from peripherals */
#define J721E_PADS_BANK0_GPIO_IE             (1 << 6)  /* Input enable */
#define J721E_PADS_BANK0_GPIO_DRIVE_SHIFT    (4)       /* Drive strength. */
#define J721E_PADS_BANK0_GPIO_DRIVE_MASK     (0x03 << J721E_PADS_BANK0_GPIO_DRIVE_SHIFT)
#define J721E_PADS_BANK0_GPIO_DRIVE_2MA      (0x0 << J721E_PADS_BANK0_GPIO_DRIVE_SHIFT)
#define J721E_PADS_BANK0_GPIO_DRIVE_4MA      (0x1 << J721E_PADS_BANK0_GPIO_DRIVE_SHIFT)
#define J721E_PADS_BANK0_GPIO_DRIVE_8MA      (0x2 << J721E_PADS_BANK0_GPIO_DRIVE_SHIFT)
#define J721E_PADS_BANK0_GPIO_DRIVE_12MA     (0x3 << J721E_PADS_BANK0_GPIO_DRIVE_SHIFT)
#define J721E_PADS_BANK0_GPIO_PUE            (1 << 3)  /* Pull up enable */
#define J721E_PADS_BANK0_GPIO_PDE            (1 << 2)  /* Pull down enable */
#define J721E_PADS_BANK0_GPIO_SCHMITT        (1 << 1)  /* Enable schmitt trigger */
#define J721E_PADS_BANK0_GPIO_SLEWFAST       (1 << 0)  /* Slew rate control. 1 = Fast, 0 = Slow */

#endif /* __ARCH_ARM_SRC_J721E_HARDWARE_J721E_PADS_BANK0_H */
