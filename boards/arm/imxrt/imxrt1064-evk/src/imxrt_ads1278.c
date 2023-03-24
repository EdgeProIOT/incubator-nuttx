/****************************************************************************
 * boards/arm/imxrt/imxrt1064-evk/src/imxrt_ads1278.c
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

#include <syslog.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/mutex.h>

#include "imxrt_config.h"
#include "imxrt_gpio.h"

#include "imxrt1064-evk.h"

#include "hardware/imxrt_pinmux.h"
#include "hardware/imxrt_flexio.h"
#include "hardware/imxrt_ccm.h"
#include "hardware/imxrt_dmamux.h"
#include "imxrt_flexio.h"
#include "imxrt_edma.h"
#include "imxrt_periphclks.h"

#include <arch/board/board.h>

#ifdef CONFIG_IMXRT_ADS1278

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FLEXIO_BCLK_PIN                 (5U)
#define FLEXIO_FRAME_SYNC_PIN           (4U)
#define FLEXIO_RX_DATA_PIN              (6U)
#define FLEXIO_BCLK_TIMER_INDEX         0
#define FLEXIO_FS_TIMER_INDEX           1
#define FLEXIO_RX_SHIFTER_INDEX         0

#define FLEXIO_SRC_CLK_HZ               30000000U
#define FLEXIO_ADS1278_WORD_WIDTH       32U
#define FLEXIO_ADS1278_SAMPLE_RATE_HZ   10190U

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imxrt_ads1278_s
{
  struct flexio_dev_s *flexio;
  mutex_t lock;                       /* Held while chip is selected for mutual exclusion */

  volatile uint32_t rx_result;        /* Result of the RX DMA */
  const uint16_t    rx_ch;            /* The RX DMA channel number */
  DMACH_HANDLE      rx_dma;           /* DMA channel handle for RX transfers */
  sem_t             rx_sem;           /* Wait for RX DMA to complete */

};


/****************************************************************************
 * Private Function Ptototypes
 ****************************************************************************/

/* DMA support */

static int         ads1278_dmarxwait(struct imxrt_ads1278_s *priv);
static inline void ads1278_dmarxwakeup(struct imxrt_ads1278_s *priv);
static void        ads1278_dmarxcallback(DMACH_HANDLE handle, void *arg,
                                         bool done, int result);
static inline void ads1278_dmarxstart(struct imxrt_ads1278_s *priv);


/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct imxrt_ads1278_s g_ads1278 =
{

  .lock   = NXMUTEX_INITIALIZER,
  .rx_ch  = IMXRT_DMACHAN_FLEXIO2,
  .rx_sem = SEM_INITIALIZER(0),
};


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ads1278_initialize
 *
 * Description:
 *   Initialize the ADS1278 I2S in its default state
 *   (Master, 32-bit, etc.)
 *
 * Input Parameters:
 *   priv - private ADS1278 device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ads1278_initialize(struct imxrt_ads1278_s *priv)
{
  struct flexio_dev_s *flexio = priv->flexio;
  struct flexio_shifter_config_s shifter_config = {0};
  struct flexio_timer_config_s timer_config     = {0};
  uint32_t tim_div  = FLEXIO_SRC_CLK_HZ /
    (FLEXIO_ADS1278_SAMPLE_RATE_HZ * FLEXIO_ADS1278_WORD_WIDTH * 2U);
  uint32_t bclk_div = 0;

  flexio->ops->reset(flexio);

  shifter_config.timer_select   = FLEXIO_BCLK_TIMER_INDEX;
  shifter_config.pin_select     = FLEXIO_RX_DATA_PIN;
  shifter_config.timer_polarity = FLEXIO_SHIFTER_TIMER_POLARITY_ON_NEGATIVE;
  shifter_config.pin_config     = FLEXIO_PIN_CONFIG_OUTPUT_DISABLED;
  shifter_config.pin_polarity   = FLEXIO_PIN_ACTIVE_HIGH;
  shifter_config.shifter_mode   = FLEXIO_SHIFTER_MODE_RECEIVE;
  shifter_config.input_source   = FLEXIO_SHIFTER_INPUT_FROM_PIN;
  shifter_config.shifter_stop   = FLEXIO_TIMER_STOP_BIT_DISABLED;
  shifter_config.shifter_Start  =
    FLEXIO_SHIFTER_START_BIT_DISABLED_LOAD_DATA_ON_ENABLE;

  flexio->ops->set_shifter_config(
    flexio,
    FLEXIO_RX_SHIFTER_INDEX,
    &shifter_config);

  if ((tim_div % 2UL) != 0UL)
    {
        tim_div += 1U;
    }

  bclk_div = ((tim_div / 2U - 1U) |
              ((FLEXIO_ADS1278_WORD_WIDTH * 2UL - 1UL) << 8U));

  /* Set Timer for bit clock */

  timer_config.trigger_select   =
    FLEXIO_TIMER_TRIGGER_SEL_SHIFTnSTAT(FLEXIO_RX_SHIFTER_INDEX);
  timer_config.trigger_polarity = FLEXIO_TIMER_TRIGGER_POLARITY_ACTIVE_LOW;
  timer_config.trigger_Source   = FLEXIO_TIMER_TRIGGER_SOURCE_INTERNAL;
  timer_config.pin_Select       = FLEXIO_BCLK_PIN;
  timer_config.pin_config       = FLEXIO_PIN_CONFIG_OUTPUT;
  timer_config.pin_polarity     = FLEXIO_PIN_ACTIVE_HIGH;
  timer_config.timer_mode       = FLEXIO_TIMER_MODE_DUAL8_BIT_BAUD_BIT;
  timer_config.timer_output     =
    FLEXIO_TIMER_OUTPUT_ONE_NOT_AFFECTED_BY_RESET;
  timer_config.timer_decrement  =
    FLEXIO_TIMER_DEC_SRC_ON_FLEX_IO_CLOCK_SHIFT_TIMER_OUTPUT;
  timer_config.timer_reset      = FLEXIO_TIMER_RESET_NEVER;
  timer_config.timer_disable    = FLEXIO_TIMER_DISABLE_NEVER;
  timer_config.timer_enable     = FLEXIO_TIMER_ENABLED_ALWAYS;
  timer_config.timer_start      = FLEXIO_TIMER_START_BIT_DISABLED;
  timer_config.timer_stop       = FLEXIO_TIMER_STOP_BIT_DISABLED;
  timer_config.timer_compare    = bclk_div;

  flexio->ops->set_timer_config(
    flexio,
    FLEXIO_BCLK_TIMER_INDEX,
    &timer_config);

  /* Set Timer for frame sync */

  timer_config.trigger_select   =
    FLEXIO_TIMER_TRIGGER_SEL_TIMn(FLEXIO_BCLK_TIMER_INDEX);
  timer_config.trigger_polarity = FLEXIO_TIMER_TRIGGER_POLARITY_ACTIVE_LOW;
  timer_config.trigger_source   = FLEXIO_TIMER_TRIGGER_SOURCE_EXTERNAL;
  timer_config.pin_config       = FLEXIO_PIN_CONFIG_OUTPUT;
  timer_config.pin_select       = FLEXIO_FRAME_SYNC_PIN;
  timer_config.pin_polarity     = FLEXIO_PIN_ACTIVE_LOW;
  timer_config.timer_mode       = FLEXIO_TIMER_MODE_SINGLE16_BIT;
  timer_config.timer_output     =
    FLEXIO_TIMER_OUTPUT_ONE_NOT_AFFECTED_BY_RESET;
  timer_config.timer_decrement  =
    FLEXIO_TIMER_DEC_SRC_ON_TRIGGER_INPUT_SHIFT_TIMER_OUTPUT;
  timer_config.timer_reset      = FLEXIO_TIMER_RESET_NEVER;
  timer_config.timer_disable    = FLEXIO_TIMER_DISABLE_NEVER;
  timer_config.timer_enable     = FLEXIO_TIMER_ENABLE_ON_PREV_TIMER_ENABLE;
  timer_config.timer_start      = FLEXIO_TIMER_START_BIT_DISABLED;
  timer_config.timer_stop       = FLEXIO_TIMER_STOP_BIT_DISABLED;
  timer_config.timer_compare    = FLEXIO_ADS1278_WORD_WIDTH * tim_div - 1U;

  flexio->ops->set_timer_config(
    flexio,
    FLEXIO_FS_TIMER_INDEX,
    &timer_config);

  flexio->ops->enable(flexio, true);
}

/****************************************************************************
 * Name: ads1278_recv (with DMA capability)
 *
 * Description:
 *   Receive a block of data from ADS1278 using DMA
 *
 * Input Parameters:
 *   priv     - Device-specific state data
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the ADS1278 interface.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ads1278_recv(
  struct imxrt_ads1278_s *priv,
  void *rxbuffer,
  size_t nwords)
{
  int       ret;
  uint32_t  regval;

  DEBUGASSERT(priv != NULL);
  
  if (rxbuffer)
    {
      up_invalidate_dcache((uintptr_t)rxbuffer,
                           (uintptr_t)rxbuffer + nwords);
    }

  /* Set up the DMA */

  struct imxrt_edma_xfrconfig_s config;

  config.saddr  = priv->spibase + IMXRT_LPSPI_RDR_OFFSET;
  config.daddr  = (uint32_t)rxbuffer;
  config.soff   = 0;
  config.doff   = 0;
  config.iter   = nwords;
  config.flags  = EDMA_CONFIG_LINKTYPE_LINKNONE;
  config.ssize  = EDMA_32BIT;
  config.dsize  = EDMA_32BIT;
  config.nbytes = 4;

  imxrt_dmach_xfrsetup(priv->rx_dma, &config);

  /* Start the DMAs */

  ads1278_dmarxstart(priv);

  /* Invoke FlexIO DMA */

  

  /* Then wait for each to complete */

  ads1278_dmarxwait(priv);

  /* Reset any status */

  /* Disable DMA */

}

/****************************************************************************
 * Name: ads1278_dmarxwait
 *
 * Description:
 *   Wait for DMA to complete.
 *
 ****************************************************************************/

static int ads1278_dmarxwait(struct imxrt_ads1278_s *priv)
{
  int ret;

  /* Take the semaphore (perhaps waiting). If the result is zero, then the
   * DMA must not really have completed.
   */

  do
    {
      ret = nxsem_wait_uninterruptible(&priv->rx_sem);

      /* The only expected error is ECANCELED which would occur if the
       * calling thread were canceled.
       */

      DEBUGASSERT(ret == OK || ret == -ECANCELED);
    }
  while (priv->rx_result == 0 && ret == OK);

  return ret;
}
#endif

/****************************************************************************
 * Name: ads1278_dmarxwakeup
 *
 * Description:
 *   Signal that DMA is complete
 *
 ****************************************************************************/

static inline void ads1278_dmarxwakeup(struct imxrt_ads1278_s *priv)
{
  nxsem_post(&priv->rx_sem);
}

/****************************************************************************
 * Name: ads1278_dmarxcallback
 *
 * Description:
 *   Called when the RX DMA completes
 *
 ****************************************************************************/

static void ads1278_dmarxcallback(DMACH_HANDLE handle, void *arg, bool done,
                                  int result)
{
  struct imxrt_ads1278_s *priv = (struct imxrt_ads1278_s *)arg;

  priv->rx_result = result | 0x80000000;  /* assure non-zero */
  ads1278_dmarxwakeup(priv);
}

/****************************************************************************
 * Name: ads1278_dmarxstart
 *
 * Description:
 *   Start RX DMA
 *
 ****************************************************************************/

static inline void ads1278_dmarxstart(struct imxrt_ads1278_s *priv)
{
  priv->rx_result = 0;
  imxrt_dmach_start(priv->rx_dma, ads1278_dmarxcallback, priv);
}



/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_ads1278_initialize
 *
 * Description:
 *   Initialize the ADS1278 device
 *
 * Returned Value:
 *   Valid device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

void imxrt_ads1278_initialize(void)
{
  struct imxrt_ads1278_s *priv = &g_ads1278;
  struct flexio_dev_s *flexio;

  imxrt_config_gpio(GPIO_FLEXIO3_FSYNC);    /* GPIO_AD_B1_04 */
  imxrt_config_gpio(GPIO_FLEXIO3_BCLK);     /* GPIO_AD_B1_05 */
  imxrt_config_gpio(GPIO_FLEXIO3_RX);       /* GPIO_AD_B1_06 */

  flexio = imxrt_flexio_initialize(3);
  if (flexio == NULL)
    {
      DEBUGASSERT(flexio_dev);
    }

  priv->flexio = flexio;

  if (priv->rx_ch)
    {
      if (priv->rx_dma == NULL)
        {
          priv->rx_dma = imxrt_dmach_alloc(priv->rx_ch | DMAMUX_CHCFG_ENBL,
                                           0);
          DEBUGASSERT(priv->rx_dma);
        }
    }
  else
    {
      priv->rx_dma = NULL;
    }

  ads1278_initialize(priv);
}


#endif /* CONFIG_IMXRT_ADS1278 */
