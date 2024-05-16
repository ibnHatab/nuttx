/****************************************************************************
 * boards/arm/j721e/common/src/j721e_common_initialize.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "j721e_gpio.h"
#include "j721e_uniqueid.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: j721e_common_earlyinitialize
 *
 * Description:
 *  This is the early initialization common to all J721E boards.
 *  It configures the UART pins so the system console can be used.
 ****************************************************************************/

void j721e_common_earlyinitialize(void)
{
  j721e_gpio_initialize();

  /* Disable IE on GPIO 26-29 */

  clrbits_reg32(J721E_PADS_BANK0_GPIO_IE, J721E_PADS_BANK0_GPIO(26));
  clrbits_reg32(J721E_PADS_BANK0_GPIO_IE, J721E_PADS_BANK0_GPIO(27));
  clrbits_reg32(J721E_PADS_BANK0_GPIO_IE, J721E_PADS_BANK0_GPIO(28));
  clrbits_reg32(J721E_PADS_BANK0_GPIO_IE, J721E_PADS_BANK0_GPIO(29));

  /* Set default UART pin */

#ifdef CONFIG_J721E_UART0
  j721e_gpio_set_function(CONFIG_J721E_UART0_TX_GPIO,
                           J721E_GPIO_FUNC_UART);      /* TX */
  j721e_gpio_set_function(CONFIG_J721E_UART0_RX_GPIO,
                           J721E_GPIO_FUNC_UART);      /* RX */
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  j721e_gpio_set_function(CONFIG_J721E_UART0_CTS_GPIO,
                           J721E_GPIO_FUNC_UART);      /* CTS */
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  j721e_gpio_set_function(CONFIG_J721E_UART0_RTS_GPIO,
                           J721E_GPIO_FUNC_UART);      /* RTS */
#endif
#endif

#ifdef CONFIG_J721E_UART1
  j721e_gpio_set_function(CONFIG_J721E_UART1_TX_GPIO,
                           J721E_GPIO_FUNC_UART);      /* TX */
  j721e_gpio_set_function(CONFIG_J721E_UART1_RX_GPIO,
                           J721E_GPIO_FUNC_UART);      /* RX */
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  j721e_gpio_set_function(CONFIG_J721E_UART1_CTS_GPIO,
                           J721E_GPIO_FUNC_UART);      /* CTS */
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  j721e_gpio_set_function(CONFIG_J721E_UART1_RTS_GPIO,
                           J721E_GPIO_FUNC_UART);      /* RTS */
#endif
#endif

#if defined(CONFIG_J721E_CLK_GPOUT0)
  j721e_gpio_set_function(J721E_GPIO_PIN_CLK_GPOUT0,
                           J721E_GPIO_FUNC_CLOCKS);
#endif
#if defined(CONFIG_J721E_CLK_GPOUT1)
  j721e_gpio_set_function(J721E_GPIO_PIN_CLK_GPOUT1,
                           J721E_GPIO_FUNC_CLOCKS);
#endif
#if defined(CONFIG_J721E_CLK_GPOUT2)
  j721e_gpio_set_function(J721E_GPIO_PIN_CLK_GPOUT2,
                           J721E_GPIO_FUNC_CLOCKS);
#endif
#if defined(CONFIG_J721E_CLK_GPOUT3)
  j721e_gpio_set_function(J721E_GPIO_PIN_CLK_GPOUT3,
                           J721E_GPIO_FUNC_CLOCKS);
#endif
}

/****************************************************************************
 * Name: j721e_common_initialize
 *
 * Description:
 *  It configures the pin assignments that were not done in the early
 *  initialization.
 ****************************************************************************/

void j721e_common_initialize(void)
{
#ifdef CONFIG_BOARDCTL_UNIQUEID
  j721e_uniqueid_initialize();
#endif

  /* Set default I2C pin */

#ifdef CONFIG_J721E_I2C0
  j721e_gpio_set_function(CONFIG_J721E_I2C0_SDA_GPIO,
                           J721E_GPIO_FUNC_I2C);       /* SDA */
  j721e_gpio_set_function(CONFIG_J721E_I2C0_SCL_GPIO,
                           J721E_GPIO_FUNC_I2C);       /* SCL */

  j721e_gpio_set_pulls(CONFIG_J721E_I2C0_SDA_GPIO, true, false);  /* Pull up */
  j721e_gpio_set_pulls(CONFIG_J721E_I2C0_SCL_GPIO, true, false);
#endif

#ifdef CONFIG_J721E_I2C1
  j721e_gpio_set_function(CONFIG_J721E_I2C1_SDA_GPIO,
                           J721E_GPIO_FUNC_I2C);       /* SDA */
  j721e_gpio_set_function(CONFIG_J721E_I2C1_SCL_GPIO,
                           J721E_GPIO_FUNC_I2C);       /* SCL */

  j721e_gpio_set_pulls(CONFIG_J721E_I2C1_SDA_GPIO, true, false);  /* Pull up */
  j721e_gpio_set_pulls(CONFIG_J721E_I2C1_SCL_GPIO, true, false);
#endif

  /* Set default SPI pin */

#ifdef CONFIG_J721E_SPI0
  j721e_gpio_set_function(CONFIG_J721E_SPI0_RX_GPIO,
                           J721E_GPIO_FUNC_SPI);       /* RX */
  j721e_gpio_set_function(CONFIG_J721E_SPI0_SCK_GPIO,
                           J721E_GPIO_FUNC_SPI);       /* SCK */
  j721e_gpio_set_function(CONFIG_J721E_SPI0_TX_GPIO,
                           J721E_GPIO_FUNC_SPI);       /* TX */

  /* CSn is controlled by board-specific logic */

  j721e_gpio_init(CONFIG_J721E_SPI0_CS_GPIO);        /* CSn */
  j721e_gpio_setdir(CONFIG_J721E_SPI0_CS_GPIO, true);
  j721e_gpio_put(CONFIG_J721E_SPI0_CS_GPIO, true);
#endif

#ifdef CONFIG_J721E_SPI1
  j721e_gpio_set_function(CONFIG_J721E_SPI1_RX_GPIO,
                           J721E_GPIO_FUNC_SPI);       /* RX */
  j721e_gpio_set_function(CONFIG_J721E_SPI1_SCK_GPIO,
                           J721E_GPIO_FUNC_SPI);       /* SCK */
  j721e_gpio_set_function(CONFIG_J721E_SPI1_TX_GPIO,
                           J721E_GPIO_FUNC_SPI);       /* TX */

  /* CSn is controlled by board-specific logic */

  j721e_gpio_init(CONFIG_J721E_SPI1_CS_GPIO);        /* CSn */
  j721e_gpio_setdir(CONFIG_J721E_SPI1_CS_GPIO, true);
  j721e_gpio_put(CONFIG_J721E_SPI1_CS_GPIO, true);
#endif

#ifdef CONFIG_NET_W5500
  /* W5500 Reset output */

  j721e_gpio_setdir(CONFIG_J721E_W5500_RST_GPIO, true);
  j721e_gpio_put(CONFIG_J721E_W5500_RST_GPIO, false);
  j721e_gpio_set_function(CONFIG_J721E_W5500_RST_GPIO,
                           J721E_GPIO_FUNC_SIO);

  /* W5500 Interrupt input */

  j721e_gpio_init(CONFIG_J721E_W5500_INT_GPIO);
#endif
}
