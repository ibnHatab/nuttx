/****************************************************************************
 * arch/arm/src/j721e/hardware/j721e_spi.h
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

#ifndef __ARCH_ARM_SRC_J721E_HARDWARE_J721E_SPI_H
#define __ARCH_ARM_SRC_J721E_HARDWARE_J721E_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/j721e_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define J721E_SPI_SSPCR0_OFFSET        0x000000  /* Control register 0 */
#define J721E_SPI_SSPCR1_OFFSET        0x000004  /* Control register 1 */
#define J721E_SPI_SSPDR_OFFSET         0x000008  /* Data register */
#define J721E_SPI_SSPSR_OFFSET         0x00000c  /* Status register */
#define J721E_SPI_SSPCPSR_OFFSET       0x000010  /* Clock prescale register */
#define J721E_SPI_SSPIMSC_OFFSET       0x000014  /* Interrupt mask set or clear register */
#define J721E_SPI_SSPRIS_OFFSET        0x000018  /* Raw interrupt status register */
#define J721E_SPI_SSPMIS_OFFSET        0x00001c  /* Masked interrupt status register */
#define J721E_SPI_SSPICR_OFFSET        0x000020  /* Interrupt clear register */
#define J721E_SPI_SSPDMACR_OFFSET      0x000024  /* DMA control register */
#define J721E_SPI_SSPPERIPHID0_OFFSET  0x000fe0  /* Peripheral identification registers */
#define J721E_SPI_SSPPERIPHID1_OFFSET  0x000fe4  /* Peripheral identification registers */
#define J721E_SPI_SSPPERIPHID2_OFFSET  0x000fe8  /* Peripheral identification registers */
#define J721E_SPI_SSPPERIPHID3_OFFSET  0x000fec  /* Peripheral identification registers */
#define J721E_SPI_SSPPCELLID0_OFFSET   0x000ff0  /* PrimeCell identification registers */
#define J721E_SPI_SSPPCELLID1_OFFSET   0x000ff4  /* PrimeCell identification registers */
#define J721E_SPI_SSPPCELLID2_OFFSET   0x000ff8  /* PrimeCell identification registers */
#define J721E_SPI_SSPPCELLID3_OFFSET   0x000ffc  /* PrimeCell identification registers */

/* Register definitions *****************************************************/

#define J721E_SPI_SSPCR0(n)        (J721E_SPI_BASE(n) + J721E_SPI_SSPCR0_OFFSET)
#define J721E_SPI_SSPCR1(n)        (J721E_SPI_BASE(n) + J721E_SPI_SSPCR1_OFFSET)
#define J721E_SPI_SSPDR(n)         (J721E_SPI_BASE(n) + J721E_SPI_SSPDR_OFFSET)
#define J721E_SPI_SSPSR(n)         (J721E_SPI_BASE(n) + J721E_SPI_SSPSR_OFFSET)
#define J721E_SPI_SSPCPSR(n)       (J721E_SPI_BASE(n) + J721E_SPI_SSPCPSR_OFFSET)
#define J721E_SPI_SSPIMSC(n)       (J721E_SPI_BASE(n) + J721E_SPI_SSPIMSC_OFFSET)
#define J721E_SPI_SSPRIS(n)        (J721E_SPI_BASE(n) + J721E_SPI_SSPRIS_OFFSET)
#define J721E_SPI_SSPMIS(n)        (J721E_SPI_BASE(n) + J721E_SPI_SSPMIS_OFFSET)
#define J721E_SPI_SSPICR(n)        (J721E_SPI_BASE(n) + J721E_SPI_SSPICR_OFFSET)
#define J721E_SPI_SSPDMACR(n)      (J721E_SPI_BASE(n) + J721E_SPI_SSPDMACR_OFFSET)
#define J721E_SPI_SSPPERIPHID0(n)  (J721E_SPI_BASE(n) + J721E_SPI_SSPPERIPHID0_OFFSET)
#define J721E_SPI_SSPPERIPHID1(n)  (J721E_SPI_BASE(n) + J721E_SPI_SSPPERIPHID1_OFFSET)
#define J721E_SPI_SSPPERIPHID2(n)  (J721E_SPI_BASE(n) + J721E_SPI_SSPPERIPHID2_OFFSET)
#define J721E_SPI_SSPPERIPHID3(n)  (J721E_SPI_BASE(n) + J721E_SPI_SSPPERIPHID3_OFFSET)
#define J721E_SPI_SSPPCELLID0(n)   (J721E_SPI_BASE(n) + J721E_SPI_SSPPCELLID0_OFFSET)
#define J721E_SPI_SSPPCELLID1(n)   (J721E_SPI_BASE(n) + J721E_SPI_SSPPCELLID1_OFFSET)
#define J721E_SPI_SSPPCELLID2(n)   (J721E_SPI_BASE(n) + J721E_SPI_SSPPCELLID2_OFFSET)
#define J721E_SPI_SSPPCELLID3(n)   (J721E_SPI_BASE(n) + J721E_SPI_SSPPCELLID3_OFFSET)

/* Register bit definitions *************************************************/

#define J721E_SPI_SSPCR0_SCR_SHIFT                 (8)       /* Serial clock rate */
#define J721E_SPI_SSPCR0_SCR_MASK                  (0xff << J721E_SPI_SSPCR0_SCR_SHIFT)
#define J721E_SPI_SSPCR0_SPH                       (1 << 7)  /* SSPCLKOUT phase */
#define J721E_SPI_SSPCR0_SPO                       (1 << 6)  /* SSPCLKOUT polarity */
#define J721E_SPI_SSPCR0_FRF_SHIFT                 (4)       /* Frame format */
#define J721E_SPI_SSPCR0_FRF_MASK                  (0x03 << J721E_SPI_SSPCR0_FRF_SHIFT)
#define J721E_SPI_SSPCR0_DSS_MASK                  (0x0f)    /* Data Size Select */
#define J721E_SPI_SSPCR0_DSS_SHIFT                 (0)

#define J721E_SPI_SSPCR1_SOD                       (1 << 3)  /* Slave-mode output disable */
#define J721E_SPI_SSPCR1_MS                        (1 << 2)  /* Master or slave mode select */
#define J721E_SPI_SSPCR1_SSE                       (1 << 1)  /* Synchronous serial port enable: 0 SSP operation disabled. 1 SSP operation enabled. */
#define J721E_SPI_SSPCR1_LBM                       (1 << 0)  /* Loop back mode */

#define J721E_SPI_SSPDR_DATA_MASK                  (0xffff)  /* Transmit/Receive FIFO */

#define J721E_SPI_SSPSR_BSY                        (1 << 4)  /* PrimeCell SSP busy flag */
#define J721E_SPI_SSPSR_RFF                        (1 << 3)  /* Receive FIFO full */
#define J721E_SPI_SSPSR_RNE                        (1 << 2)  /* Receive FIFO not empty */
#define J721E_SPI_SSPSR_TNF                        (1 << 1)  /* Transmit FIFO not full */
#define J721E_SPI_SSPSR_TFE                        (1 << 0)  /* Transmit FIFO empty */

#define J721E_SPI_SSPCPSR_CPSDVSR_MASK             (0xff)    /* Clock prescale divisor. Must be an even number from 2-254 */

#define J721E_SPI_SSPIMSC_TXIM                     (1 << 3)  /* Transmit FIFO interrupt mask */
#define J721E_SPI_SSPIMSC_RXIM                     (1 << 2)  /* Receive FIFO interrupt mask */
#define J721E_SPI_SSPIMSC_RTIM                     (1 << 1)  /* Receive timeout interrupt mask */
#define J721E_SPI_SSPIMSC_RORIM                    (1 << 0)  /* Receive overrun interrupt mask */

#define J721E_SPI_SSPRIS_TXRIS                     (1 << 3)  /* Gives the raw interrupt state, prior to masking, of the SSPTXINTR interrupt */
#define J721E_SPI_SSPRIS_RXRIS                     (1 << 2)  /* Gives the raw interrupt state, prior to masking, of the SSPRXINTR interrupt */
#define J721E_SPI_SSPRIS_RTRIS                     (1 << 1)  /* Gives the raw interrupt state, prior to masking, of the SSPRTINTR interrupt */
#define J721E_SPI_SSPRIS_RORRIS                    (1 << 0)  /* Gives the raw interrupt state, prior to masking, of the SSPRORINTR interrupt */

#define J721E_SPI_SSPMIS_TXMIS                     (1 << 3)  /* Gives the transmit FIFO masked interrupt state, after masking, of the SSPTXINTR interrupt */
#define J721E_SPI_SSPMIS_RXMIS                     (1 << 2)  /* Gives the receive FIFO masked interrupt state, after masking, of the SSPRXINTR interrupt */
#define J721E_SPI_SSPMIS_RTMIS                     (1 << 1)  /* Gives the receive timeout masked interrupt state, after masking, of the SSPRTINTR interrupt */
#define J721E_SPI_SSPMIS_RORMIS                    (1 << 0)  /* Gives the receive over run masked interrupt status, after masking, of the SSPRORINTR interrupt */

#define J721E_SPI_SSPICR_RTIC                      (1 << 1)  /* Clears the SSPRTINTR interrupt */
#define J721E_SPI_SSPICR_RORIC                     (1 << 0)  /* Clears the SSPRORINTR interrupt */

#define J721E_SPI_SSPDMACR_TXDMAE                  (1 << 1)  /* Transmit DMA Enable. If this bit is set to 1, DMA for the transmit FIFO is enabled. */
#define J721E_SPI_SSPDMACR_RXDMAE                  (1 << 0)  /* Receive DMA Enable. If this bit is set to 1, DMA for the receive FIFO is enabled. */

#define J721E_SPI_SSPPERIPHID0_PARTNUMBER0_MASK    (0xff)  /* These bits read back as 0x22 */

#define J721E_SPI_SSPPERIPHID1_DESIGNER0_SHIFT     (4)  /* These bits read back as 0x1 */
#define J721E_SPI_SSPPERIPHID1_DESIGNER0_MASK      (0x0f << J721E_SPI_SSPPERIPHID1_DESIGNER0_SHIFT)
#define J721E_SPI_SSPPERIPHID1_PARTNUMBER1_MASK    (0x0f)  /* These bits read back as 0x0 */

#define J721E_SPI_SSPPERIPHID2_REVISION_SHIFT      (4)  /* These bits return the peripheral revision */
#define J721E_SPI_SSPPERIPHID2_REVISION_MASK       (0x0f << J721E_SPI_SSPPERIPHID2_REVISION_SHIFT)
#define J721E_SPI_SSPPERIPHID2_DESIGNER1_MASK      (0x0f)  /* These bits read back as 0x4 */

#define J721E_SPI_SSPPERIPHID3_CONFIGURATION_MASK  (0xff)  /* These bits read back as 0x00 */

#define J721E_SPI_SSPPCELLID0_MASK                 (0xff)  /* These bits read back as 0x0D */

#define J721E_SPI_SSPPCELLID1_MASK                 (0xff)  /* These bits read back as 0xF0 */

#define J721E_SPI_SSPPCELLID2_MASK                 (0xff)  /* These bits read back as 0x05 */

#define J721E_SPI_SSPPCELLID3_MASK                 (0xff)  /* These bits read back as 0xB1 */

#endif /* __ARCH_ARM_SRC_J721E_HARDWARE_J721E_SPI_H */
