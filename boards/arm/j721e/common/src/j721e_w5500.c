/****************************************************************************
 * boards/arm/j721e/common/src/j721e_w5500.c
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

#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/net/w5500.h>

#include "j721e_spi.h"
#include "j721e_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Sanity Check *************************************************************/

#if (CONFIG_J721E_W5500_SPI_CH == 0) && !defined(CONFIG_J721E_SPI0)
# error "W5500 configured to use SPI0, but SPI0 is not enabled"
#endif

#if (CONFIG_J721E_W5500_SPI_CH == 1) && !defined(CONFIG_J721E_SPI1)
# error "W5500 configured to use SPI1, but SPI1 is not enabled"
#endif

/* SPI Assumptions **********************************************************/

#define W5500_DEVNO      0   /* Only one W5500 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct j721e_lower_s
{
  const struct w5500_lower_s lower;    /* Low-level MCU interface */
  xcpt_t                     handler;  /* W5500 interrupt handler */
  void                      *arg;      /* Argument that accompanies IRQ */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  j721e_attach(const struct w5500_lower_s *lower, xcpt_t handler,
                          void *arg);
static void j721e_enable(const struct w5500_lower_s *lower, bool enable);
static void j721e_reset(const struct w5500_lower_s *lower, bool reset);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct j721e_lower_s g_enclower =
{
  .lower =
  {
    .frequency = (CONFIG_J721E_W5500_SPI_FREQ * 1000),
    .spidevid  = 0,
    .mode      = SPIDEV_MODE0,
    .attach    = j721e_attach,
    .enable    = j721e_enable,
    .reset     = j721e_reset,
  },
  .handler = NULL,
  .arg     = NULL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: j721e_attach
 *
 * Description:
 *   Attaches the interrupt handler to the GPIO.
 *
 * Input Parameters:
 *   lower - W5500 lower half
 *   handler - The handler function
 *   arg - Argument to pass to handler
 *
 * Returned Value:
 *   Zero (OK) is returned on success.
 *
 ****************************************************************************/

static int j721e_attach(const struct w5500_lower_s *lower,
                         xcpt_t handler,
                         void *arg)
{
  struct j721e_lower_s *priv = (struct j721e_lower_s *)lower;

  priv->handler = handler;
  priv->arg     = arg;
  j721e_gpio_irq_attach(CONFIG_J721E_W5500_INT_GPIO,
                         J721E_GPIO_INTR_LEVEL_LOW,
                         priv->handler, priv->arg);
  return OK;
}

/****************************************************************************
 * Name: j721e_enable
 *
 * Description:
 *   Enables the W5500 interrupt handler.
 *
 * Input Parameters:
 *   lower - W5500 lower half
 *   enable - true to enable, false to disable
 *
 ****************************************************************************/

static void j721e_enable(const struct w5500_lower_s *lower, bool enable)
{
  struct j721e_lower_s *priv = (struct j721e_lower_s *)lower;

  DEBUGASSERT(priv->handler);
  if (enable)
    {
      j721e_gpio_enable_irq(CONFIG_J721E_W5500_INT_GPIO);
    }
  else
    {
      j721e_gpio_disable_irq(CONFIG_J721E_W5500_INT_GPIO);
    }
}

/****************************************************************************
 * Name: j721e_reset
 *
 * Description:
 *   Brings the W5500 in or out of reset.
 *
 * Input Parameters:
 *   lower - W5500 lower half
 *   reset - true to reset, false to enable
 *
 ****************************************************************************/

static void j721e_reset(const struct w5500_lower_s *lower, bool reset)
{
  j721e_gpio_put(CONFIG_J721E_W5500_RST_GPIO, !reset);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_netinitialize
 *
 * Description:
 *   Initializes the SPI and W5500 drivers.
 *
 ****************************************************************************/

void arm_netinitialize(void)
{
  struct spi_dev_s *spi;
  int ret;

  spi = j721e_spibus_initialize(CONFIG_J721E_W5500_SPI_CH);
  if (!spi)
    {
      nerr("ERROR: Failed to initialize SPI port %d\n",
           CONFIG_J721E_W5500_SPI_CH);
      return;
    }

  ret = w5500_initialize(spi, &g_enclower.lower, W5500_DEVNO);
  if (ret < 0)
    {
      nerr("ERROR: Failed to bind SPI%d W5500 device %d: %d\n",
           CONFIG_J721E_W5500_SPI_CH, W5500_DEVNO, ret);
      return;
    }

  ninfo("Bound SPI%d to W5500 device %d\n",
        CONFIG_J721E_W5500_SPI_CH, W5500_DEVNO);
}

