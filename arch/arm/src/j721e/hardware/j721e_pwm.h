/****************************************************************************
 * arch/arm/src/j721e/hardware/j721e_pwm.h
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

#ifndef __ARCH_ARM_SRC_J721E_HARDWARE_J721E_PWM_H
#define __ARCH_ARM_SRC_J721E_HARDWARE_J721E_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/j721e_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define J721E_PWM_CSR_OFFSET(n)  (0x000000 + (n) * 20) /* PWM control and status register */
#define J721E_PWM_DIV_OFFSET(n)  (0x000004 + (n) * 20) /* PWM clock divisor register */
#define J721E_PWM_CTR_OFFSET(n)  (0x000008 + (n) * 20) /* PWM counter register */
#define J721E_PWM_CC_OFFSET(n)   (0x00000C + (n) * 20) /* PWM compare register */
#define J721E_PWM_TOP_OFFSET(n)  (0x000010 + (n) * 20) /* PWM wrap value register */
#define J721E_PWM_ENA_OFFSET     0x0000A0              /* PWM enable register */
#define J721E_PWM_INTR_OFFSET    0x0000A4              /* PWM raw interrupt register */
#define J721E_PWM_INTE_OFFSET    0x0000A8              /* PWM interrupt enable register */
#define J721E_PWM_INTF_OFFSET    0x0000AC              /* PWM interrupt force register */
#define J721E_PWM_INTS_OFFSET    0x0000B0              /* PWM interrupt status register */

/* Register definitions *****************************************************/

#define J721E_PWM_CSR(n)  (J721E_PWM_BASE + J721E_PWM_CSR_OFFSET(n))
#define J721E_PWM_DIV(n)  (J721E_PWM_BASE + J721E_PWM_DIV_OFFSET(n))
#define J721E_PWM_CTR(n)  (J721E_PWM_BASE + J721E_PWM_CTR_OFFSET(n))
#define J721E_PWM_CC(n)   (J721E_PWM_BASE + J721E_PWM_CC_OFFSET(n))
#define J721E_PWM_TOP(n)  (J721E_PWM_BASE + J721E_PWM_TOP_OFFSET(n))
#define J721E_PWM_ENA     (J721E_PWM_BASE + J721E_PWM_ENA_OFFSET)
#define J721E_PWM_INTR    (J721E_PWM_BASE + J721E_PWM_INTR_OFFSET)
#define J721E_PWM_INTE    (J721E_PWM_BASE + J721E_PWM_INTE_OFFSET)
#define J721E_PWM_INTF    (J721E_PWM_BASE + J721E_PWM_INTF_OFFSET)
#define J721E_PWM_INTS    (J721E_PWM_BASE + J721E_PWM_INTS_OFFSET)

/* Register bit definitions *************************************************/

#define J721E_PWM_CSR_PH_ADV        (1 << 7) /* advance phase of counter by one */
#define J721E_PWM_CSR_PH_RET        (1 << 5) /* retard phase of counter by one */
#define J721E_PWM_CSR_DIVMODE_SHIFT (4)      /* divisor mode */
#define J721E_PWM_CSR_DIVMODE_MASK  (0x03 << J721E_PWM_CSR_DIVMODE_SHIFT)
#define J721E_PWM_CSR_B_INV         (1 << 3) /* invert output B */
#define J721E_PWM_CSR_A_INV         (1 << 2) /* invert output A */
#define J721E_PWM_CSR_PH_CORRECT    (1 << 1) /* enable phase correct modulation */
#define J721E_PWM_CSR_EN            (1 << 0) /* enable the PWM channel */

#define J721E_PWN_CSR_DIVMODE_DIV    0x00
#define J721E_PWN_CSR_DIVMODE_LEVEL  0x01
#define J721E_PWN_CSR_DIVMODE_RISE   0x02
#define J721E_PWN_CSR_DIVMODE_FALL   0x03
#define J721E_PWM_DIV_INT_SHIFT     (4)      /* divisor integer part */
#define J721E_PWM_DIV_INT_MASK      (0xff << J721E_PWM_DIV_INT_SHIFT)
#define J721E_PWM_DIV_FRAC_SHIFT    (0)      /* divisor fraction part */
#define J721E_PWM_DIV_FRAC_MASK     (0x0f << J721E_PWM_DIV_FRAC_SHIFT)

#define J721E_PWM_CC_B_SHIFT        (16)      /* channel B compare register */
#define J721E_PWM_CC_B_MASK         (0xffff << J721E_PWM_CC_B_SHIFT)
#define J721E_PWM_CC_A_SHIFT        (0)       /* channel A compare register */
#define J721E_PWM_CC_A_MASK         (0xffff << J721E_PWM_CC_A_SHIFT)

#define J721E_PWM_TOP_SHIFT         (0)       /* channel A compare register */
#define J721E_PWM_TOP_MASK          (0xffff << J721E_PWM_TOP_SHIFT)

/*  Bit mask for ENA, INTR, INTE, INTF, and INTS registers */

#define J721E_PWM_CH7              (1 << 7) /* PWM channel 7 */
#define J721E_PWM_CH6              (1 << 6) /* PWM channel 6 */
#define J721E_PWM_CH5              (1 << 5) /* PWM channel 5 */
#define J721E_PWM_CH4              (1 << 4) /* PWM channel 4 */
#define J721E_PWM_CH3              (1 << 3) /* PWM channel 3 */
#define J721E_PWM_CH2              (1 << 2) /* PWM channel 2 */
#define J721E_PWM_CH1              (1 << 1) /* PWM channel 1 */
#define J721E_PWM_CH0              (1 << 0) /* PWM channel 0 */

/****************************************************************************
 * The following IOCTL values set additional flags in the J721E PWM
 * device.
 ****************************************************************************/

/****************************************************************************
 * PWMIOC_J721E_SETINVERTPULSE sets the pulse invert flag.
 *
 * The argument is an integer where:
 *   bit zero is set to invert channel A
 *   bit one  is set to invert channel B
 ****************************************************************************/

#define PWMIOC_J721E_SETINVERTPULSE  _PWMIOC(0x80)

#define PWMIOC_J721E_GETINVERTPULSE  _PWMIOC(0x81)

/****************************************************************************
 * PWMIOC_J721E_SETPHASECORRECT sets phase correct flags.
 *
 * The argument is an integer which if non-zero sets the phase correct flag.
 ****************************************************************************/

#define PWMIOC_J721E_SETPHASECORRECT _PWMIOC(0x82)

#define PWMIOC_J721E_GETPHASECORRECT _PWMIOC(0x83)

#endif