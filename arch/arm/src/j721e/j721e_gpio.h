/****************************************************************************
 * arch/arm/src/j721e/j721e_gpio.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_J721E_J721E_GPIO_H
#define __ARCH_ARM_SRC_J721E_J721E_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include "hardware/j721e_sio.h"
#include "hardware/j721e_io_bank0.h"
#include "hardware/j721e_pads_bank0.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define J721E_GPIO_NUM    30       /* Number of GPIO pins */

/* GPIO function types ******************************************************/

#define J721E_GPIO_FUNC_JTAG       J721E_IO_BANK0_GPIO_CTRL_FUNCSEL_JTAG
#define J721E_GPIO_FUNC_SPI        J721E_IO_BANK0_GPIO_CTRL_FUNCSEL_SPI
#define J721E_GPIO_FUNC_UART       J721E_IO_BANK0_GPIO_CTRL_FUNCSEL_UART
#define J721E_GPIO_FUNC_I2C        J721E_IO_BANK0_GPIO_CTRL_FUNCSEL_I2C
#define J721E_GPIO_FUNC_PWM        J721E_IO_BANK0_GPIO_CTRL_FUNCSEL_PWM
#define J721E_GPIO_FUNC_SIO        J721E_IO_BANK0_GPIO_CTRL_FUNCSEL_SIO
#define J721E_GPIO_FUNC_PIO0       J721E_IO_BANK0_GPIO_CTRL_FUNCSEL_PIO0
#define J721E_GPIO_FUNC_PIO1       J721E_IO_BANK0_GPIO_CTRL_FUNCSEL_PIO1
#define J721E_GPIO_FUNC_CLOCKS     J721E_IO_BANK0_GPIO_CTRL_FUNCSEL_CLOCKS
#define J721E_GPIO_FUNC_USB        J721E_IO_BANK0_GPIO_CTRL_FUNCSEL_USB
#define J721E_GPIO_FUNC_NULL       J721E_IO_BANK0_GPIO_CTRL_FUNCSEL_NULL

/* GPIO function pins *******************************************************/

#define J721E_GPIO_PIN_CLK_GPOUT0  (21)
#define J721E_GPIO_PIN_CLK_GPOUT1  (23)
#define J721E_GPIO_PIN_CLK_GPOUT2  (24)
#define J721E_GPIO_PIN_CLK_GPOUT3  (25)

/* GPIO interrupt modes *****************************************************/

#define J721E_GPIO_INTR_LEVEL_LOW  0
#define J721E_GPIO_INTR_LEVEL_HIGH 1
#define J721E_GPIO_INTR_EDGE_LOW   2
#define J721E_GPIO_INTR_EDGE_HIGH  3

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline void j721e_gpio_put(uint32_t gpio, int set)
{
  uint32_t value = 1 << gpio;

  DEBUGASSERT(gpio < J721E_GPIO_NUM);

  if (set)
    {
      putreg32(value, J721E_SIO_GPIO_OUT_SET);
    }
  else
    {
      putreg32(value, J721E_SIO_GPIO_OUT_CLR);
    }
}

static inline bool j721e_gpio_get(uint32_t gpio)
{
  uint32_t value = 1 << gpio;

  DEBUGASSERT(gpio < J721E_GPIO_NUM);

  return (getreg32(J721E_SIO_GPIO_IN) & value) != 0;
}

static inline void j721e_gpio_setdir(uint32_t gpio, int out)
{
  uint32_t value = 1 << gpio;

  DEBUGASSERT(gpio < J721E_GPIO_NUM);

  if (out)
    {
      putreg32(value, J721E_SIO_GPIO_OE_SET);
    }
  else
    {
      putreg32(value, J721E_SIO_GPIO_OE_CLR);
    }
}

/****************************************************************************
 * Name: j721e_gpio_set_input_hysteresis_enabled
 *
 * Description:
 *   Set whether the pin's input hysteresis will be enabled.
 *
 ****************************************************************************/

static inline void j721e_gpio_set_input_hysteresis_enabled(uint32_t gpio,
                                                            bool enabled)
{
  DEBUGASSERT(gpio < J721E_GPIO_NUM);

  modbits_reg32(enabled ? J721E_PADS_BANK0_GPIO_SCHMITT : 0,
                J721E_PADS_BANK0_GPIO_SCHMITT,
                J721E_PADS_BANK0_GPIO(gpio));
}

/****************************************************************************
 * Name: j721e_gpio_set_slew_fast
 *
 * Description:
 *   Set whether the pin's fast slew rate will be enabled.
 *
 ****************************************************************************/

static inline void j721e_gpio_set_slew_fast(uint32_t gpio,
                                             bool enabled)
{
  DEBUGASSERT(gpio < J721E_GPIO_NUM);

  modbits_reg32(enabled ? J721E_PADS_BANK0_GPIO_SLEWFAST : 0,
                J721E_PADS_BANK0_GPIO_SLEWFAST,
                J721E_PADS_BANK0_GPIO(gpio));
}

/****************************************************************************
 * Name: j721e_gpio_set_drive_strength
 *
 * Description:
 *   Set the pin's drive strength.
 *
 ****************************************************************************/

static inline void j721e_gpio_set_drive_strength(uint32_t gpio,
                                                  uint32_t drive_strength)
{
  DEBUGASSERT(gpio < J721E_GPIO_NUM);

  modbits_reg32(drive_strength,
                J721E_PADS_BANK0_GPIO_DRIVE_MASK,
                J721E_PADS_BANK0_GPIO(gpio));
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: r2040_gpio_get_function_pin
 *
 * Description:
 *   Get the GPIO pin number to which the specified function is assigned
 *
 ****************************************************************************/

int j721e_gpio_get_function_pin(uint32_t func, uint32_t port);

/****************************************************************************
 * Name: r2040_gpio_set_function
 *
 * Description:
 *   Assign functions to the specified GPIO pin
 *
 ****************************************************************************/

void j721e_gpio_set_function(uint32_t gpio, uint32_t func);

/****************************************************************************
 * Name: r2040_gpio_set_pulls
 *
 * Description:
 *   Set pull-up or pull-down to the specified GPIO pin
 *
 ****************************************************************************/

void j721e_gpio_set_pulls(uint32_t gpio, int up, int down);

/****************************************************************************
 * Name: r2040_gpio_init
 *
 * Description:
 *   Initialize software-controlled GPIO function
 *
 ****************************************************************************/

void j721e_gpio_init(uint32_t gpio);

/****************************************************************************
 * Name: r2040_gpio_irq_attach
 *
 * Description:
 *   Configure the interrupt generated by the specified GPIO pin.
 *
 ****************************************************************************/

int j721e_gpio_irq_attach(uint32_t gpio, uint32_t intrmode,
                           xcpt_t isr, void *arg);

/****************************************************************************
 * Name: j721e_gpio_enable_irq
 *
 * Description:
 *   Enable the GPIO IRQ specified by 'gpio'
 *
 ****************************************************************************/

void j721e_gpio_enable_irq(uint32_t gpio);

/****************************************************************************
 * Name: j721e_gpio_disable_irq
 *
 * Description:
 *   Disable the GPIO IRQ specified by 'gpio'
 *
 ****************************************************************************/

void j721e_gpio_disable_irq(uint32_t gpio);

/****************************************************************************
 * Name: j721e_gpio_clear_interrupt
 *
 * Description:
 *   Clear the interrupt flags for a gpio pin.
 *
 ****************************************************************************/

void j721e_gpio_clear_interrupt(uint32_t gpio,
                                 bool     edge_low,
                                 bool     edge_high);

/****************************************************************************
 * Name: r2040_gpio_initialize
 *
 * Description:
 *   Initialize GPIO function management
 *
 ****************************************************************************/

void j721e_gpio_initialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_J721E_J721E_GPIO_H */
