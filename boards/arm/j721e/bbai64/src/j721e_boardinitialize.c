/****************************************************************************
 * boards/arm/j721e/bbai64/src/j721e_boardinitialize.c
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

#ifdef CONFIG_ARCH_BOARD_COMMON
#include "j721e_common_initialize.h"
#endif /* CONFIG_ARCH_BOARD_COMMON */

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
 * Name: j721e_boardearlyinitialize
 *
 * Description:
 *
 ****************************************************************************/

void j721e_boardearlyinitialize(void)
{
  #ifdef CONFIG_ARCH_BOARD_COMMON
  j721e_common_earlyinitialize();
  #endif

  /* --- Place any board specific early initialization here --- */

  /* Set board LED pin */

  j721e_gpio_init(BOARD_GPIO_LED_PIN);
  j721e_gpio_setdir(BOARD_GPIO_LED_PIN, true);
  j721e_gpio_put(BOARD_GPIO_LED_PIN, true);
}

/****************************************************************************
 * Name: j721e_boardinitialize
 *
 * Description:
 *
 ****************************************************************************/

void j721e_boardinitialize(void)
{
  #ifdef CONFIG_ARCH_BOARD_COMMON
  j721e_common_initialize();
  #endif

  /* --- Place any board specific initialization here --- */
}
