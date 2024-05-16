/****************************************************************************
 * arch/arm/src/j721e/hardware/j721e_dma.h
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

#ifndef __ARCH_ARM_SRC_J721E_HARDWARE_J721E_DMA_H
#define __ARCH_ARM_SRC_J721E_HARDWARE_J721E_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/j721e_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define J721E_DMA_READ_ADDR_OFFSET                  0x000000  /* DMA Read Address pointer */
#define J721E_DMA_WRITE_ADDR_OFFSET                 0x000004  /* DMA Write Address pointer */
#define J721E_DMA_TRANS_COUNT_OFFSET                0x000008  /* DMA Transfer Count */
#define J721E_DMA_CTRL_TRIG_OFFSET                  0x00000c  /* DMA Control and Status */
#define J721E_DMA_AL1_CTRL_OFFSET                   0x000010  /* Alias for CTRL register */
#define J721E_DMA_AL1_READ_ADDR_OFFSET              0x000014  /* Alias for READ_ADDR register */
#define J721E_DMA_AL1_WRITE_ADDR_OFFSET             0x000018  /* Alias for WRITE_ADDR register */
#define J721E_DMA_AL1_TRANS_COUNT_TRIG_OFFSET       0x00001c  /* Alias for TRANS_COUNT register */
#define J721E_DMA_AL2_CTRL_OFFSET                   0x000020  /* Alias for CTRL register */
#define J721E_DMA_AL2_TRANS_COUNT_OFFSET            0x000024  /* Alias for TRANS_COUNT register */
#define J721E_DMA_AL2_READ_ADDR_OFFSET              0x000028  /* Alias for READ_ADDR register */
#define J721E_DMA_AL2_WRITE_ADDR_TRIG_OFFSET        0x00002c  /* Alias for WRITE_ADDR register */
#define J721E_DMA_AL3_CTRL_OFFSET                   0x000030  /* Alias for CTRL register */
#define J721E_DMA_AL3_WRITE_ADDR_OFFSET             0x000034  /* Alias for WRITE_ADDR register */
#define J721E_DMA_AL3_TRANS_COUNT_OFFSET            0x000038  /* Alias for TRANS_COUNT register */
#define J721E_DMA_AL3_READ_ADDR_TRIG_OFFSET         0x00003c  /* Alias for READ_ADDR register */

#define J721E_DMA_INTR_OFFSET                       0x000400  /* Interrupt Status (raw) */
#define J721E_DMA_INTE0_OFFSET                      0x000404  /* Interrupt Enables for IRQ 0 */
#define J721E_DMA_INTF0_OFFSET                      0x000408  /* Force Interrupts */
#define J721E_DMA_INTS0_OFFSET                      0x00040c  /* Interrupt Status for IRQ 0 */
#define J721E_DMA_INTE1_OFFSET                      0x000414  /* Interrupt Enables for IRQ 1 */
#define J721E_DMA_INTF1_OFFSET                      0x000418  /* Force Interrupts for IRQ 1 */
#define J721E_DMA_INTS1_OFFSET                      0x00041c  /* Interrupt Status (masked) for IRQ 1 */
#define J721E_DMA_TIMER0_OFFSET                     0x000420  /* Pacing (X/Y) Fractional Timer The pacing timer produces TREQ assertions at a rate set by ((X/Y) * sys_clk). This equation is evaluated every sys_clk cycles and therefore can only generate TREQs at a rate of 1 per sys_clk (i.e. permanent TREQ) or less. */
#define J721E_DMA_TIMER1_OFFSET                     0x000424  /* Pacing (X/Y) Fractional Timer The pacing timer produces TREQ assertions at a rate set by ((X/Y) * sys_clk). This equation is evaluated every sys_clk cycles and therefore can only generate TREQs at a rate of 1 per sys_clk (i.e. permanent TREQ) or less. */
#define J721E_DMA_TIMER2_OFFSET                     0x000428  /* Pacing (X/Y) Fractional Timer The pacing timer produces TREQ assertions at a rate set by ((X/Y) * sys_clk). This equation is evaluated every sys_clk cycles and therefore can only generate TREQs at a rate of 1 per sys_clk (i.e. permanent TREQ) or less. */
#define J721E_DMA_TIMER3_OFFSET                     0x00042c  /* Pacing (X/Y) Fractional Timer The pacing timer produces TREQ assertions at a rate set by ((X/Y) * sys_clk). This equation is evaluated every sys_clk cycles and therefore can only generate TREQs at a rate of 1 per sys_clk (i.e. permanent TREQ) or less. */
#define J721E_DMA_MULTI_CHAN_TRIGGER_OFFSET         0x000430  /* Trigger one or more channels simultaneously */
#define J721E_DMA_SNIFF_CTRL_OFFSET                 0x000434  /* Sniffer Control */
#define J721E_DMA_SNIFF_DATA_OFFSET                 0x000438  /* Data accumulator for sniff hardware Write an initial seed value here before starting a DMA transfer on the channel indicated by SNIFF_CTRL_DMACH. The hardware will update this register each time it observes a read from the indicated channel. Once the channel completes, the final result can be read from this register. */
#define J721E_DMA_FIFO_LEVELS_OFFSET                0x000440  /* Debug RAF, WAF, TDF levels */
#define J721E_DMA_CHAN_ABORT_OFFSET                 0x000444  /* Abort an in-progress transfer sequence on one or more channels */
#define J721E_DMA_N_CHANNELS_OFFSET                 0x000448  /* The number of channels this DMA instance is equipped with. This DMA supports up to 16 hardware channels, but can be configured with as few as one, to minimise silicon area. */
#define J721E_DMA_DBG_CTDREQ_OFFSET(n)              (0x000800 + (n) * 0x0040)
#define J721E_DMA_DBG_TCR_OFFSET                    (0x000804 + (n) * 0x0040)

/* Register definitions *****************************************************/

#define J721E_DMA_CH(n)                      (J721E_DMA_BASE + (0x0040 * (n)))
#define J721E_DMA_READ_ADDR(n)               (J721E_DMA_CH(n) + J721E_DMA_READ_ADDR_OFFSET)
#define J721E_DMA_WRITE_ADDR(n)              (J721E_DMA_CH(n) + J721E_DMA_WRITE_ADDR_OFFSET)
#define J721E_DMA_TRANS_COUNT(n)             (J721E_DMA_CH(n) + J721E_DMA_TRANS_COUNT_OFFSET)
#define J721E_DMA_CTRL_TRIG(n)               (J721E_DMA_CH(n) + J721E_DMA_CTRL_TRIG_OFFSET)
#define J721E_DMA_AL1_CTRL(n)                (J721E_DMA_CH(n) + J721E_DMA_AL1_CTRL_OFFSET)
#define J721E_DMA_AL1_READ_ADDR(n)           (J721E_DMA_CH(n) + J721E_DMA_AL1_READ_ADDR_OFFSET)
#define J721E_DMA_AL1_WRITE_ADDR(n)          (J721E_DMA_CH(n) + J721E_DMA_AL1_WRITE_ADDR_OFFSET)
#define J721E_DMA_AL1_TRANS_COUNT_TRIG(n)    (J721E_DMA_CH(n) + J721E_DMA_AL1_TRANS_COUNT_TRIG_OFFSET)
#define J721E_DMA_AL2_CTRL(n)                (J721E_DMA_CH(n) + J721E_DMA_AL2_CTRL_OFFSET)
#define J721E_DMA_AL2_TRANS_COUNT(n)         (J721E_DMA_CH(n) + J721E_DMA_AL2_TRANS_COUNT_OFFSET)
#define J721E_DMA_AL2_READ_ADDR(n)           (J721E_DMA_CH(n) + J721E_DMA_AL2_READ_ADDR_OFFSET)
#define J721E_DMA_AL2_WRITE_ADDR_TRIG(n)     (J721E_DMA_CH(n) + J721E_DMA_AL2_WRITE_ADDR_TRIG_OFFSET)
#define J721E_DMA_AL3_CTRL(n)                (J721E_DMA_CH(n) + J721E_DMA_AL3_CTRL_OFFSET)
#define J721E_DMA_AL3_WRITE_ADDR(n)          (J721E_DMA_CH(n) + J721E_DMA_AL3_WRITE_ADDR_OFFSET)
#define J721E_DMA_AL3_TRANS_COUNT(n)         (J721E_DMA_CH(n) + J721E_DMA_AL3_TRANS_COUNT_OFFSET)
#define J721E_DMA_AL3_READ_ADDR_TRIG(n)      (J721E_DMA_CH(n) + J721E_DMA_AL3_READ_ADDR_TRIG_OFFSET)

#define J721E_DMA_INTR                       (J721E_DMA_BASE + J721E_DMA_INTR_OFFSET)
#define J721E_DMA_INTE0                      (J721E_DMA_BASE + J721E_DMA_INTE0_OFFSET)
#define J721E_DMA_INTF0                      (J721E_DMA_BASE + J721E_DMA_INTF0_OFFSET)
#define J721E_DMA_INTS0                      (J721E_DMA_BASE + J721E_DMA_INTS0_OFFSET)
#define J721E_DMA_INTE1                      (J721E_DMA_BASE + J721E_DMA_INTE1_OFFSET)
#define J721E_DMA_INTF1                      (J721E_DMA_BASE + J721E_DMA_INTF1_OFFSET)
#define J721E_DMA_INTS1                      (J721E_DMA_BASE + J721E_DMA_INTS1_OFFSET)
#define J721E_DMA_TIMER0                     (J721E_DMA_BASE + J721E_DMA_TIMER0_OFFSET)
#define J721E_DMA_TIMER1                     (J721E_DMA_BASE + J721E_DMA_TIMER1_OFFSET)
#define J721E_DMA_TIMER2                     (J721E_DMA_BASE + J721E_DMA_TIMER2_OFFSET)
#define J721E_DMA_TIMER3                     (J721E_DMA_BASE + J721E_DMA_TIMER3_OFFSET)
#define J721E_DMA_MULTI_CHAN_TRIGGER         (J721E_DMA_BASE + J721E_DMA_MULTI_CHAN_TRIGGER_OFFSET)
#define J721E_DMA_SNIFF_CTRL                 (J721E_DMA_BASE + J721E_DMA_SNIFF_CTRL_OFFSET)
#define J721E_DMA_SNIFF_DATA                 (J721E_DMA_BASE + J721E_DMA_SNIFF_DATA_OFFSET)
#define J721E_DMA_FIFO_LEVELS                (J721E_DMA_BASE + J721E_DMA_FIFO_LEVELS_OFFSET)
#define J721E_DMA_CHAN_ABORT                 (J721E_DMA_BASE + J721E_DMA_CHAN_ABORT_OFFSET)
#define J721E_DMA_N_CHANNELS                 (J721E_DMA_BASE + J721E_DMA_N_CHANNELS_OFFSET)

#define J721E_DMA_DBG_CTDREQ(n)              (J721E_DMA_BASE + J721E_DMA_DBG_CTDREQ_OFFSET(n))
#define J721E_DMA_DBG_TCR(n)                 (J721E_DMA_BASE + J721E_DMA_DBG_TCR_OFFSET(n))

/* Register bit definitions *************************************************/

#define J721E_DMA_CTRL_TRIG_AHB_ERROR                     (1 << 31)  /* Logical OR of the READ_ERROR and WRITE_ERROR flags. The channel halts when it encounters any bus error, and always raises its channel IRQ flag. */
#define J721E_DMA_CTRL_TRIG_READ_ERROR                    (1 << 30)  /* If 1, the channel received a read bus error. Write one to clear. READ_ADDR shows the approximate address where the bus error was encountered (will not to be earlier, or more than 3 transfers later) */
#define J721E_DMA_CTRL_TRIG_WRITE_ERROR                   (1 << 29)  /* If 1, the channel received a write bus error. Write one to clear. WRITE_ADDR shows the approximate address where the bus error was encountered (will not to be earlier, or more than 5 transfers later) */
#define J721E_DMA_CTRL_TRIG_BUSY                          (1 << 24)  /* This flag goes high when the channel starts a new transfer sequence, and low when the last transfer of that sequence completes. Clearing EN while BUSY is high pauses the channel, and BUSY will stay high while paused.  To terminate a sequence early (and clear the BUSY flag), see CHAN_ABORT. */
#define J721E_DMA_CTRL_TRIG_SNIFF_EN                      (1 << 23)  /* If 1, this channel's data transfers are visible to the sniff hardware, and each transfer will advance the state of the checksum. This only applies if the sniff hardware is enabled, and has this channel selected.  This allows checksum to be enabled or disabled on a per-control- block basis. */
#define J721E_DMA_CTRL_TRIG_BSWAP                         (1 << 22)  /* Apply byte-swap transformation to DMA data. For byte data, this has no effect. For halfword data, the two bytes of each halfword are swapped. For word data, the four bytes of each word are swapped to reverse order. */
#define J721E_DMA_CTRL_TRIG_IRQ_QUIET                     (1 << 21)  /* In QUIET mode, the channel does not generate IRQs at the end of every transfer block. Instead, an IRQ is raised when NULL is written to a trigger register, indicating the end of a control block chain.  This reduces the number of interrupts to be serviced by the CPU when transferring a DMA chain of many small control blocks. */
#define J721E_DMA_CTRL_TRIG_TREQ_SEL_SHIFT                (15)       /* Select a Transfer Request signal. The channel uses the transfer request signal to pace its data transfer rate. Sources for TREQ signals are internal (TIMERS) or external (DREQ, a Data Request from the system). 0x0 to 0x3a -> select DREQ n as TREQ */
#define J721E_DMA_CTRL_TRIG_TREQ_SEL_MASK                 (0x3f << J721E_DMA_CTRL_TRIG_TREQ_SEL_SHIFT)
#define J721E_DMA_CTRL_TRIG_TREQ_SEL_TIMER0               (0x3b << J721E_DMA_CTRL_TRIG_TREQ_SEL_SHIFT)  /* Select Timer 0 as TREQ */
#define J721E_DMA_CTRL_TRIG_TREQ_SEL_TIMER1               (0x3c << J721E_DMA_CTRL_TRIG_TREQ_SEL_SHIFT)  /* Select Timer 1 as TREQ */
#define J721E_DMA_CTRL_TRIG_TREQ_SEL_TIMER2               (0x3d << J721E_DMA_CTRL_TRIG_TREQ_SEL_SHIFT)  /* Select Timer 2 as TREQ (Optional) */
#define J721E_DMA_CTRL_TRIG_TREQ_SEL_TIMER3               (0x3e << J721E_DMA_CTRL_TRIG_TREQ_SEL_SHIFT)  /* Select Timer 3 as TREQ (Optional) */
#define J721E_DMA_CTRL_TRIG_TREQ_SEL_PERMANENT            (0x3f << J721E_DMA_CTRL_TRIG_TREQ_SEL_SHIFT)  /* Permanent request, for unpaced transfers. */
#define J721E_DMA_CTRL_TRIG_CHAIN_TO_SHIFT                (11)                                           /* When this channel completes, it will trigger the channel indicated by CHAIN_TO. Disable by setting CHAIN_TO = _(this channel)_. Reset value is equal to channel number (0). */
#define J721E_DMA_CTRL_TRIG_CHAIN_TO_MASK                 (0x0f << J721E_DMA_CTRL_TRIG_CHAIN_TO_SHIFT)
#define J721E_DMA_CTRL_TRIG_RING_SEL                      (1 << 10)  /* Select whether RING_SIZE applies to read or write addresses. If 0, read addresses are wrapped on a (1 << RING_SIZE) boundary. If 1, write addresses are wrapped. */
#define J721E_DMA_CTRL_TRIG_RING_SIZE_SHIFT               (6)        /* Size of address wrap region. If 0, don't wrap. For values n > 0, only the lower n bits of the address will change. This wraps the address on a (1 << n) byte boundary, facilitating access to naturally-aligned ring buffers.  Ring sizes between 2 and 32768 bytes are possible. This can apply to either read or write addresses, based on value of RING_SEL. */
#define J721E_DMA_CTRL_TRIG_RING_SIZE_MASK                (0x0f << J721E_DMA_CTRL_TRIG_RING_SIZE_SHIFT)
#define J721E_DMA_CTRL_TRIG_RING_SIZE_RING_NONE           (0x0 << J721E_DMA_CTRL_TRIG_RING_SIZE_SHIFT)
#define J721E_DMA_CTRL_TRIG_INCR_WRITE                    (1 << 5)   /* If 1, the write address increments with each transfer. If 0, each write is directed to the same, initial address.  Generally this should be disabled for memory-to-peripheral transfers. */
#define J721E_DMA_CTRL_TRIG_INCR_READ                     (1 << 4)   /* If 1, the read address increments with each transfer. If 0, each read is directed to the same, initial address.  Generally this should be disabled for peripheral-to-memory transfers. */
#define J721E_DMA_CTRL_TRIG_DATA_SIZE_SHIFT               (2)        /* Set the size of each bus transfer (byte/halfword/word). READ_ADDR and WRITE_ADDR advance by this amount (1/2/4 bytes) with each transfer. */
#define J721E_DMA_CTRL_TRIG_DATA_SIZE_MASK                (0x03 << J721E_DMA_CTRL_TRIG_DATA_SIZE_SHIFT)
#define J721E_DMA_CTRL_TRIG_DATA_SIZE_SIZE_BYTE           (0x0 << J721E_DMA_CTRL_TRIG_DATA_SIZE_SHIFT)
#define J721E_DMA_CTRL_TRIG_DATA_SIZE_SIZE_HALFWORD       (0x1 << J721E_DMA_CTRL_TRIG_DATA_SIZE_SHIFT)
#define J721E_DMA_CTRL_TRIG_DATA_SIZE_SIZE_WORD           (0x2 << J721E_DMA_CTRL_TRIG_DATA_SIZE_SHIFT)
#define J721E_DMA_CTRL_TRIG_HIGH_PRIORITY                 (1 << 1)   /* HIGH_PRIORITY gives a channel preferential treatment in issue scheduling: in each scheduling round, all high priority channels are considered first, and then only a single low priority channel, before returning to the high priority channels.  This only affects the order in which the DMA schedules channels. The DMA's bus priority is not changed. If the DMA is not saturated then a low priority channel will see no loss of throughput. */
#define J721E_DMA_CTRL_TRIG_EN                            (1 << 0)   /* DMA Channel Enable. When 1, the channel will respond to triggering events, which will cause it to become BUSY and start transferring data. When 0, the channel will ignore triggers, stop issuing transfers, and pause the current transfer sequence (i.e. BUSY will remain high if already high) */

#define J721E_DMA_INTR_MASK                               (0xffff)  /* Raw interrupt status for DMA Channels 0..15. Bit n corresponds to channel n. Ignores any masking or forcing. Channel interrupts can be cleared by writing a bit mask to INTR, INTS0 or INTS1.  Channel interrupts can be routed to either of two system-level IRQs based on INTE0 and INTE1.  This can be used vector different channel interrupts to different ISRs: this might be done to allow NVIC IRQ preemption for more time-critical channels, or to spread IRQ load across different cores.  It is also valid to ignore this behaviour and just use INTE0/INTS0/IRQ 0. */

#define J721E_DMA_INTE0_MASK                              (0xffff)  /* Set bit n to pass interrupts from channel n to DMA IRQ 0. */

#define J721E_DMA_INTF0_MASK                              (0xffff)  /* Write 1s to force the corresponding bits in INTE0. The interrupt remains asserted until INTF0 is cleared. */

#define J721E_DMA_INTS0_MASK                              (0xffff)  /* Indicates active channel interrupt requests which are currently causing IRQ 0 to be asserted. Channel interrupts can be cleared by writing a bit mask here. */

#define J721E_DMA_INTE1_MASK                              (0xffff)  /* Set bit n to pass interrupts from channel n to DMA IRQ 1. */

#define J721E_DMA_INTF1_MASK                              (0xffff)  /* Write 1s to force the corresponding bits in INTE0. The interrupt remains asserted until INTF0 is cleared. */

#define J721E_DMA_INTS1_MASK                              (0xffff)  /* Indicates active channel interrupt requests which are currently causing IRQ 1 to be asserted. Channel interrupts can be cleared by writing a bit mask here. */

#define J721E_DMA_TIMER0_X_SHIFT                          (16)  /* Pacing Timer Dividend. Specifies the X value for the (X/Y) fractional timer. */
#define J721E_DMA_TIMER0_X_MASK                           (0xffff << J721E_DMA_TIMER0_X_SHIFT)
#define J721E_DMA_TIMER0_Y_MASK                           (0xffff)  /* Pacing Timer Divisor. Specifies the Y value for the (X/Y) fractional timer. */

#define J721E_DMA_TIMER1_X_SHIFT                          (16)  /* Pacing Timer Dividend. Specifies the X value for the (X/Y) fractional timer. */
#define J721E_DMA_TIMER1_X_MASK                           (0xffff << J721E_DMA_TIMER1_X_SHIFT)
#define J721E_DMA_TIMER1_Y_MASK                           (0xffff)  /* Pacing Timer Divisor. Specifies the Y value for the (X/Y) fractional timer. */

#define J721E_DMA_TIMER2_X_SHIFT                          (16)  /* Pacing Timer Dividend. Specifies the X value for the (X/Y) fractional timer. */
#define J721E_DMA_TIMER2_X_MASK                           (0xffff << J721E_DMA_TIMER2_X_SHIFT)
#define J721E_DMA_TIMER2_Y_MASK                           (0xffff)  /* Pacing Timer Divisor. Specifies the Y value for the (X/Y) fractional timer. */

#define J721E_DMA_TIMER3_X_SHIFT                          (16)       /* Pacing Timer Dividend. Specifies the X value for the (X/Y) fractional timer. */
#define J721E_DMA_TIMER3_X_MASK                           (0xffff << J721E_DMA_TIMER3_X_SHIFT)
#define J721E_DMA_TIMER3_Y_MASK                           (0xffff)   /* Pacing Timer Divisor. Specifies the Y value for the (X/Y) fractional timer. */

#define J721E_DMA_MULTI_CHAN_TRIGGER_MASK                 (0xffff)   /* Each bit in this register corresponds to a DMA channel. Writing a 1 to the relevant bit is the same as writing to that channel's trigger register; the channel will start if it is currently enabled and not already busy. */

#define J721E_DMA_SNIFF_CTRL_OUT_INV                      (1 << 11)  /* If set, the result appears inverted (bitwise complement) when read. This does not affect the way the checksum is calculated; the result is transformed on-the-fly between the result register and the bus. */
#define J721E_DMA_SNIFF_CTRL_OUT_REV                      (1 << 10)  /* If set, the result appears bit-reversed when read. This does not affect the way the checksum is calculated; the result is transformed on-the-fly between the result register and the bus. */
#define J721E_DMA_SNIFF_CTRL_BSWAP                        (1 << 9)   /* Locally perform a byte reverse on the sniffed data, before feeding into checksum.  Note that the sniff hardware is downstream of the DMA channel byteswap performed in the read master: if channel CTRL_BSWAP and SNIFF_CTRL_BSWAP are both enabled, their effects cancel from the sniffer's point of view. */
#define J721E_DMA_SNIFF_CTRL_CALC_SHIFT                   (5)
#define J721E_DMA_SNIFF_CTRL_CALC_MASK                    (0x0f << J721E_DMA_SNIFF_CTRL_CALC_SHIFT)
#define J721E_DMA_SNIFF_CTRL_CALC_CRC32                   (0x0 << J721E_DMA_SNIFF_CTRL_CALC_SHIFT)  /* Calculate a CRC-32 (IEEE802.3 polynomial) */
#define J721E_DMA_SNIFF_CTRL_CALC_CRC32R                  (0x1 << J721E_DMA_SNIFF_CTRL_CALC_SHIFT)  /* Calculate a CRC-32 (IEEE802.3 polynomial) with bit reversed data */
#define J721E_DMA_SNIFF_CTRL_CALC_CRC16                   (0x2 << J721E_DMA_SNIFF_CTRL_CALC_SHIFT)  /* Calculate a CRC-16-CCITT */
#define J721E_DMA_SNIFF_CTRL_CALC_CRC16R                  (0x3 << J721E_DMA_SNIFF_CTRL_CALC_SHIFT)  /* Calculate a CRC-16-CCITT with bit reversed data */
#define J721E_DMA_SNIFF_CTRL_CALC_EVEN                    (0xe << J721E_DMA_SNIFF_CTRL_CALC_SHIFT)  /* XOR reduction over all data. == 1 if the total 1 population count is odd. */
#define J721E_DMA_SNIFF_CTRL_CALC_SUM                     (0xf << J721E_DMA_SNIFF_CTRL_CALC_SHIFT)  /* Calculate a simple 32-bit checksum (addition with a 32 bit accumulator) */
#define J721E_DMA_SNIFF_CTRL_DMACH_SHIFT                  (1)                                        /* DMA channel for Sniffer to observe */
#define J721E_DMA_SNIFF_CTRL_DMACH_MASK                   (0x0f << J721E_DMA_SNIFF_CTRL_DMACH_SHIFT)
#define J721E_DMA_SNIFF_CTRL_EN                           (1 << 0)                                   /* Enable sniffer */

#define J721E_DMA_FIFO_LEVELS_RAF_LVL_SHIFT               (16)      /* Current Read-Address-FIFO fill level */
#define J721E_DMA_FIFO_LEVELS_RAF_LVL_MASK                (0xff << J721E_DMA_FIFO_LEVELS_RAF_LVL_SHIFT)
#define J721E_DMA_FIFO_LEVELS_WAF_LVL_SHIFT               (8)       /* Current Write-Address-FIFO fill level */
#define J721E_DMA_FIFO_LEVELS_WAF_LVL_MASK                (0xff << J721E_DMA_FIFO_LEVELS_WAF_LVL_SHIFT)
#define J721E_DMA_FIFO_LEVELS_TDF_LVL_MASK                (0xff)    /* Current Transfer-Data-FIFO fill level */

#define J721E_DMA_CHAN_ABORT_MASK                         (0xffff)  /* Each bit corresponds to a channel. Writing a 1 aborts whatever transfer sequence is in progress on that channel. The bit will remain high until any in-flight transfers have been flushed through the address and data FIFOs.  After writing, this register must be polled until it returns all-zero. Until this point, it is unsafe to restart the channel. */

#define J721E_DMA_N_CHANNELS_MASK                         (0x1f)

#define J721E_DMA_DBG_CTDREQ_MASK                         (0x3f)

#endif /* __ARCH_ARM_SRC_J721E_HARDWARE_J721E_DMA_H */
