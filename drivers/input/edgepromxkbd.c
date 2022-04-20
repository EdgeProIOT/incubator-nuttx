/****************************************************************************
 * drivers/input/edgepromxkbd.c
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

#include <stdbool.h>
#include <stdio.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <poll.h>
#include <fcntl.h>

#include <nuttx/input/edgepromxkbd.h>
#include <nuttx/input/djoystick.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This format is used to construct the /dev/kbd[n] device driver path. It is
 * defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT          "/dev/kbd%c"
#define DEV_NAMELEN         11



#define KEY_PRESS       0x01
#define KEY_PRESS_HOLD  0x02
#define KEY_RELEASE     0x03

/* Special Key Encodings */

#define KEY_BUTTON_FIRST 0x01  /* Start of the button region */
#define KEY_ROT_SWITCH   0x01
#define KEY_BTN_LEFT1    0x02
#define KEY_BTN_RIGHT1   0x03
#define KEY_BACKSPACE    0x04  /* Normal ASCII */
#define KEY_TAB          0x05  /* Normal ASCII */
#define KEY_NL           0x06  /* Normal ASCII */
#define KEY_BUTTON_LAST  0x06  /* End of the button region */


/****************************************************************************
 * Private Types
 ****************************************************************************/

struct edgepromxkbd_dev_s
{
  FAR const struct edgepromxkbd_config_s *config;  /* Board configuration data */
  FAR struct i2c_master_s *i2c; /* Saved I2C driver instance */

  sem_t  exclsem;      /* Exclusive access to dev */
  sem_t  waitsem;      /* Signal waiting thread */
  bool   waiting;      /* Waiting for keyboard data */
  struct work_s work;  /* Supports the interrupt handling "bottom half" */

  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

  struct pollfd *fds[CONFIG_EDGEPROMXKBD_NPOLLWAITERS];

  /* Buffer used to collect and buffer incoming keyboard characters */

  uint16_t  headndx;      /* Buffer head index */
  uint16_t  tailndx;      /* Buffer tail index */
  uint8_t   kbdbuffer[CONFIG_EDGEPROMXKBD_BUFSIZE];

  uint8_t   crefs;        /* Reference count on the driver instance */
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

static int edgepromxkbd_interrupt(int irq, FAR void *context, FAR void *arg);
static void edgepromxkbd_worker(FAR void *arg);
static void edgepromxkbd_putreg8(FAR struct edgepromxkbd_dev_s *priv,
                             uint8_t regaddr, uint8_t regval);
static uint8_t edgepromxkbd_getreg8(FAR struct edgepromxkbd_dev_s *priv,
                                uint8_t regaddr);
static uint16_t edgepromxkbd_getreg16(FAR struct edgepromxkbd_dev_s *priv,
                                  uint8_t regaddr);
static int edgepromxkbd_checkver(FAR struct edgepromxkbd_dev_s *priv);
static int edgepromxkbd_reset(FAR struct edgepromxkbd_dev_s *priv);
static void edgepromxkbd_putbuffer(FAR struct edgepromxkbd_dev_s *priv,
                               uint8_t keycode);

/* Driver methods. We export the keyboard as a standard character driver */

static int  edgepromxkbd_open(FAR struct file *filep);
static int  edgepromxkbd_close(FAR struct file *filep);
static ssize_t edgepromxkbd_read(FAR struct file *filep,
                             FAR char *buffer, size_t len);
static ssize_t edgepromxkbd_write(FAR struct file *filep,
                              FAR const char *buffer, size_t len);
static int  edgepromxkbd_poll(FAR struct file *filep, FAR struct pollfd *fds,
                          bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_hidkbd_fops =
{
  edgepromxkbd_open,             /* open */
  edgepromxkbd_close,            /* close */
  edgepromxkbd_read,             /* read */
  edgepromxkbd_write,            /* write */
  NULL,                      /* seek */
  NULL,                      /* ioctl */
  edgepromxkbd_poll              /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                     /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: edgepromxkbd_worker
 ****************************************************************************/

static void edgepromxkbd_worker(FAR void *arg)
{
  FAR struct edgepromxkbd_dev_s  *priv = (FAR struct edgepromxkbd_dev_s *)arg;
  uint16_t                    regval;
  uint8_t                     key;
  uint8_t                     state;
  int                         ret;

  ret = nxsem_wait_uninterruptible(&priv->exclsem);
  if (ret < 0)
    {
      return;
    }

  regval = edgepromxkbd_getreg8(priv, EDGEPROMXKBD_INT);
  if (regval & EDGEPROMXKBD_INT_KEY)
    {
      /* There is a keypress in the FIFO */

      while (edgepromxkbd_getreg8(priv, EDGEPROMXKBD_KEY) & EDGEPROMXKBD_KEY_COUNT_MASK)
        {
          regval = edgepromxkbd_getreg16(priv, EDGEPROMXKBD_FIF);
          state = (regval & EDGEPROMXKBD_FIF_STATE_MASK) >> \
                  EDGEPROMXKBD_FIF_STATE_SHIFT;
          key = (regval & EDGEPROMXKBD_FIF_KEY_MASK) >> EDGEPROMXKBD_FIF_KEY_SHIFT;
          if (key <= KEY_BUTTON_LAST &&
              !(key == KEY_BACKSPACE ||
                key == KEY_TAB ||
                key == KEY_NL))
            {
              iinfo("Button Ignored. No joystick interface.\n");
            }
          else if(state == KEY_PRESS)
            {
              /* key is a normal ascii character */

              edgepromxkbd_putbuffer(priv, key);

              /* Notify waiting reads */

              if (priv->waiting == true)
                {
                  priv->waiting = false;
                  nxsem_post(&priv->waitsem);
                }
            }
        }
    }

  /* Clear interrupt status register */

  edgepromxkbd_putreg8(priv, EDGEPROMXKBD_INT, 0);
  nxsem_post(&priv->exclsem);
}

/****************************************************************************
 * Name: edgepromxkbd_interrupt
 ****************************************************************************/

static int edgepromxkbd_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct edgepromxkbd_dev_s  *priv = (FAR struct edgepromxkbd_dev_s *)arg;
  int                         ret;

  /* Let the event worker know that it has an interrupt event to handle
   * It is possbile that we will already have work scheduled from a
   * previous interrupt event.  That is OK we will service all the events
   * in the same work job.
   */

  if (work_available(&priv->work))
    {
      ret = work_queue(HPWORK, &priv->work, edgepromxkbd_worker, priv, 0);
      if (ret != 0)
        {
          ierr("ERROR: Failed to queue work: %d\n", ret);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: edgepromxkbd_pollnotify
 ****************************************************************************/

static void edgepromxkbd_pollnotify(FAR struct edgepromxkbd_dev_s *priv)
{
  int i;

  for (i = 0; i < CONFIG_EDGEPROMXKBD_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = priv->fds[i];
      if (fds)
        {
          fds->revents |= (fds->events & POLLIN);
          if (fds->revents != 0)
            {
              uinfo("Report events: %02x\n", fds->revents);
              nxsem_post(fds->sem);
            }
        }
    }
}

/****************************************************************************
 * Name: edgepromxkbd_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int edgepromxkbd_open(FAR struct file *filep)
{
  FAR struct inode           *inode;
  FAR struct edgepromxkbd_dev_s  *priv;

  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Increment the reference count on the driver */

  priv->crefs++;

  return OK;
}

/****************************************************************************
 * Name: edgepromxkbd_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int edgepromxkbd_close(FAR struct file *filep)
{
  FAR struct inode           *inode;
  FAR struct edgepromxkbd_dev_s  *priv;

  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Decrement the reference count on the driver */

  DEBUGASSERT(priv->crefs >= 1);

  priv->crefs--;

  return OK;
}

/****************************************************************************
 * Name: edgepromxkbd_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t edgepromxkbd_read(FAR struct file *filep, FAR char *buffer,
                             size_t len)
{
  FAR struct inode           *inode;
  FAR struct edgepromxkbd_dev_s  *priv;
  size_t                      nbytes;
  uint16_t                    tail;
  int                         ret;

  DEBUGASSERT(filep && filep->f_inode && buffer);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Read data from our internal buffer of received characters */

  ret = nxsem_wait_uninterruptible(&priv->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  while (priv->tailndx == priv->headndx)
    {
      /* No.. were we open non-blocking? */

      if (filep->f_oflags & O_NONBLOCK)
        {
          /* Yes.. then return a failure */

          ret = -EAGAIN;
          goto errout;
        }
      else
        {
          priv->waiting = true;
          nxsem_post(&priv->exclsem);
          ret = nxsem_wait_uninterruptible(&priv->waitsem);
          if (ret < 0)
            {
              return ret;
            }

          ret = nxsem_wait_uninterruptible(&priv->exclsem);
          if (ret < 0)
            {
              return ret;
            }
        }
    }

  for (tail = priv->tailndx, nbytes = 0;
       tail != priv->headndx && nbytes < len;
       nbytes++)
    {
      /* Copy the next keyboard character into the user buffer */

      *buffer++ = priv->kbdbuffer[tail];

      /* Handle wrap-around of the tail index */

      if (++tail >= CONFIG_EDGEPROMXKBD_BUFSIZE)
        {
          tail = 0;
        }
    }

  ret = nbytes;

  /* Update the tail index (perhaps marking the buffer empty) */

  priv->tailndx = tail;

errout:
  nxsem_post(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * Name: edgepromxkbd_write
 *
 * Description:
 *   Standard character driver write method.
 *
 ****************************************************************************/

static ssize_t edgepromxkbd_write(FAR struct file *filep, FAR const char *buffer,
                              size_t len)
{
  /* We won't try to write to the keyboard */

  return -ENOSYS;
}

/****************************************************************************
 * Name: edgepromxkbd_poll
 *
 * Description:
 *   Standard character driver poll method.
 *
 ****************************************************************************/

static int edgepromxkbd_poll(FAR struct file *filep, FAR struct pollfd *fds,
                         bool setup)
{
  FAR struct inode           *inode;
  FAR struct edgepromxkbd_dev_s  *priv;
  int                         ret;
  int                         i;

  DEBUGASSERT(filep && filep->f_inode && fds);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Make sure that we have exclusive access to the private data structure */

  DEBUGASSERT(priv);
  ret = nxsem_wait_uninterruptible(&priv->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      /* This is a request to set up the poll.  Find an available slot for
       * the poll structure reference
       */

      for (i = 0; i < CONFIG_EDGEPROMXKBD_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!priv->fds[i])
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv    = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_EDGEPROMXKBD_NPOLLWAITERS)
        {
          fds->priv    = NULL;
          ret          = -EBUSY;
          goto errout;
        }

      /* Should we immediately notify on any of the requested events? Notify
       * the POLLIN event if there is buffered keyboard data.
       */

      if (priv->headndx != priv->tailndx)
        {
          edgepromxkbd_pollnotify(priv);
        }
    }
  else
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;
      DEBUGASSERT(slot);

      /* Remove all memory of the poll setup */

      *slot                = NULL;
      fds->priv            = NULL;
    }

errout:
  nxsem_post(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * Name: edgepromxkbd_putbuffer
 *
 * Description:
 *   Add one character to the user buffer.
 *   Expectation is that we already have exclusive use of the device.
 *
 * Input Parameters:
 *   priv - Driver internal state
 *   keycode - The value to add to the user buffer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void edgepromxkbd_putbuffer(FAR struct edgepromxkbd_dev_s *priv,
                               uint8_t keycode)
{
  uint16_t head;
  uint16_t tail;

  DEBUGASSERT(priv);

  /* Copy the next keyboard character into the user buffer. */

  head = priv->headndx;
  priv->kbdbuffer[head] = keycode;

  /* Increment the head index */

  if (++head >= CONFIG_EDGEPROMXKBD_BUFSIZE)
    {
      head = 0;
    }

  /* If the buffer is full, then increment the tail index to make space.
   * Drop old unread key presses.
   */

  tail = priv->tailndx;
  if (tail == head)
    {
      if (++tail >= CONFIG_EDGEPROMXKBD_BUFSIZE)
        {
          tail = 0;
        }

      /* Save the updated tail index */

      priv->tailndx = tail;
    }

  /* Save the updated head index */

  priv->headndx = head;
}

/****************************************************************************
 * Name: edgepromxkbd_checkver
 *
 * Description:
 *   Read and verify the Q10 Keyboard Controller Version
 *
 ****************************************************************************/

static int edgepromxkbd_checkver(FAR struct edgepromxkbd_dev_s *priv)
{
  uint8_t version;

  /* Read device version  */

  version = edgepromxkbd_getreg8(priv, EDGEPROMXKBD_VER);
  iinfo("version: %02x\n", version);

  if (version != EDGEPROMXKBD_VER_00_02)
    {
      /* Version is not Correct */

      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: edgepromxkbd_reset
 *
 * Description:
 *   Reset the Q10 Keyboard Controller Version
 *
 ****************************************************************************/

static int edgepromxkbd_reset(FAR struct edgepromxkbd_dev_s *priv)
{
  edgepromxkbd_putreg8(priv, EDGEPROMXKBD_RST, 0xff);
  return OK;
}

/****************************************************************************
 * Name: edgepromxkbd_getreg8
 *
 * Description:
 *   Read from an 8-bit Q10 Keyboard register
 *
 ****************************************************************************/

static uint8_t edgepromxkbd_getreg8(FAR struct edgepromxkbd_dev_s *priv,
                                uint8_t regaddr)
{
  /* 8-bit data read sequence:
   *
   *  Start - I2C_Write_Address - Q10_Reg_Address -
   *    Repeated_Start - I2C_Read_Address  - Q10_Read_Data - STOP
   */

  struct i2c_msg_s msg[2];
  uint8_t          regval;
  int              ret;

  /* Setup 8-bit Q10 Keyboard address write message */

  msg[0].frequency = priv->config->frequency;  /* I2C frequency */
  msg[0].addr      = priv->config->address;    /* 7-bit address */
  msg[0].flags     = 0;                        /* Write transaction, beginning with START */
  msg[0].buffer    = &regaddr;                 /* Transfer from this address */
  msg[0].length    = 1;                        /* Send one byte following the address
                                                * (no STOP) */

  /* Set up the 8-bit Q10 Keyboard data read message */

  msg[1].frequency = priv->config->frequency;  /* I2C frequency */
  msg[1].addr      = priv->config->address;    /* 7-bit address */
  msg[1].flags     = I2C_M_READ;               /* Read transaction, beginning with Re-START */
  msg[1].buffer    = &regval;                  /* Transfer to this address */
  msg[1].length    = 1;                        /* Receive one byte following the address
                                                * (then STOP) */

  /* Perform the transfer */

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      ierr("ERROR: I2C_TRANSFER failed: %d\n", ret);
      return 0;
    }

  return regval;
}

/****************************************************************************
 * Name: edgepromxkbd_getreg16
 *
 * Description:
 *   Read from an 8-bit Q10 Keyboard register
 *
 ****************************************************************************/

static uint16_t edgepromxkbd_getreg16(FAR struct edgepromxkbd_dev_s *priv,
                                  uint8_t regaddr)
{
  /* 8-bit data read sequence:
   *
   *  Start - I2C_Write_Address - Q10_Reg_Address -
   *    Repeated_Start - I2C_Read_Address  - Q10_Read_Data - STOP
   */

  struct i2c_msg_s msg[2];
  uint8_t          regval[2];
  uint16_t              ret;

  /* Setup 8-bit Q10 Keyboard address write message */

  msg[0].frequency = priv->config->frequency;  /* I2C frequency */
  msg[0].addr      = priv->config->address;    /* 7-bit address */
  msg[0].flags     = 0;                        /* Write transaction, beginning with START */
  msg[0].buffer    = &regaddr;                 /* Transfer from this address */
  msg[0].length    = 1;                        /* Send one byte following the address
                                                * (no STOP) */

  /* Set up the 8-bit Q10 Keyboard data read message */

  msg[1].frequency = priv->config->frequency;  /* I2C frequency */
  msg[1].addr      = priv->config->address;    /* 7-bit address */
  msg[1].flags     = I2C_M_READ;               /* Read transaction, beginning with Re-START */
  msg[1].buffer    = regval;                   /* Transfer to this address */
  msg[1].length    = 2;                        /* Receive two bytes following the address
                                                * (then STOP) */

  /* Perform the transfer */

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      ierr("ERROR: I2C_TRANSFER failed: %d\n", ret);
      return 0;
    }

  ret = (regval[1] << 8) | regval[0];

  return ret;
}

/****************************************************************************
 * Name: edgepromxkbd_putreg8
 *
 * Description:
 *   Write a value to an 8-bit Q10 Keyboard register
 *
 ****************************************************************************/

static void edgepromxkbd_putreg8(FAR struct edgepromxkbd_dev_s *priv,
                             uint8_t regaddr, uint8_t regval)
{
  /* 8-bit data read sequence:
   *
   *  Start - I2C_Write_Address - Q10_Reg_Address - Q10_Write_Data - STOP
   */

  struct i2c_msg_s msg;
  uint8_t          txbuffer[2];
  int              ret;

  /* Setup to the data to be transferred.  Two bytes:  The Q10 Keyboard
   * register address followed by one byte of data.
   */

  txbuffer[0] = regaddr;
  txbuffer[1] = regval;

  /* Setup 8-bit Q10 Keyboard address write message */

  msg.frequency = priv->config->frequency;  /* I2C frequency */
  msg.addr      = priv->config->address;    /* 7-bit address */
  msg.flags     = 0;                        /* Write transaction, beginning with START */
  msg.buffer    = txbuffer;                 /* Transfer from this address */
  msg.length    = 2;                        /* Send two byte following the address
                                             * (then STOP) */

  /* Perform the transfer */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  if (ret < 0)
    {
      ierr("ERROR: I2C_TRANSFER failed: %d\n", ret);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: edgepromxkbd_register
 *
 * Description:
 *   Configure the Solder Party Q10 Keyboard to use the provided I2C device
 *   instance.  This will register the driver as /dev/kbdN where N is the
 *   minor device number, as well as a joystick at joydevname
 *
 * Input Parameters:
 *   i2c         - An I2C driver instance
 *   config      - Persistent board configuration data
 *   kbdminor    - The keyboard input device minor number
 *   joydevname  - The name of the joystick device /dev/djoyN
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int edgepromxkbd_register(FAR struct i2c_master_s *i2c,
                      FAR const struct edgepromxkbd_config_s *config,
                      char kbdminor)
{
  FAR struct edgepromxkbd_dev_s *priv;
  char                       kbddevname[DEV_NAMELEN];
  int                        ret;

  /* Debug Sanity Checks */

  DEBUGASSERT(i2c != NULL && config != NULL);
  DEBUGASSERT(config->attach != NULL && config->enable != NULL &&
              config->clear  != NULL);

  priv = (FAR struct edgepromxkbd_dev_s *)kmm_zalloc(
    sizeof(struct edgepromxkbd_dev_s));
  if (!priv)
    {
      ierr("ERROR: kmm_zalloc(%d) failed\n", sizeof(struct edgepromxkbd_dev_s));
      return -ENOMEM;
    }

  /* Initialize the device driver instance */

  priv->i2c       = i2c;     /* Save the I2C device handle */
  priv->config    = config;  /* Save the board configuration */
  priv->tailndx   = 0;       /* Reset keypress buffer state */
  priv->headndx   = 0;
  priv->crefs     = 0;       /* Reset reference count to 0 */
  priv->waiting   = false;

  nxsem_init(&priv->exclsem,  0, 1);   /* Initialize device semaphore */
  nxsem_init(&priv->waitsem, 0, 0);

  /* The waitsem semaphore is used for signaling and, hence, should
   * not have priority inheritance enabled.
   */

  nxsem_set_protocol(&priv->waitsem, SEM_PRIO_NONE);

  config->clear(config);
  config->enable(config, false);

  /* Attach the interrupt handler */

  ret = config->attach(config, edgepromxkbd_interrupt, priv);
  if (ret < 0)
    {
      ierr("ERROR: Failed to attach interrupt\n");
      goto errout_with_priv;
    }

  ret = edgepromxkbd_checkver(priv);

  if (ret != OK)
    {
      /* Did not find a supported device on the bus */

      return ret;
    }

  edgepromxkbd_reset(priv);

  /* Start servicing events */

  priv->config->enable(priv->config, true);

  snprintf(kbddevname, DEV_NAMELEN, DEV_FORMAT, kbdminor);
  iinfo("Registering %s\n", kbddevname);
  ret = register_driver(kbddevname, &g_hidkbd_fops, 0666, priv);

  return OK;

errout_with_priv:
  nxsem_destroy(&priv->exclsem);
  kmm_free(priv);
  return ret;
}
