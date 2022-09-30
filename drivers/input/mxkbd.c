/****************************************************************************
 * drivers/input/mxkbd.c
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

#include <nuttx/input/mxkbd.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/ioexpander/mcp23x17.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This format is used to construct the /dev/kbd[n] device driver path. It is
 * defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT        "/dev/kbd%c"
#define DEV_NAMELEN       11



#define KEY_PRESS         0x01
#define KEY_PRESS_HOLD    0x02
#define KEY_RELEASE       0x03

/* Special Key Encodings */

#define KEY_BUTTON_FIRST  0x01  /* Start of the button region */
#define KEY_ROT_SWITCH    0x01
#define KEY_BTN_LEFT1     0x02
#define KEY_BTN_RIGHT1    0x03
#define KEY_BACKSPACE     0x04  /* Normal ASCII */
#define KEY_TAB           0x05  /* Normal ASCII */
#define KEY_NL            0x06  /* Normal ASCII */
#define KEY_BUTTON_LAST   0x06  /* End of the button region */


/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mxkbd_dev_s
{
  FAR struct ioexpander_dev_s *dev;

  sem_t  exclsem;      /* Exclusive access to dev */
  sem_t  waitsem;      /* Signal waiting thread */
  bool   waiting;      /* Waiting for keyboard data */

  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */
  struct pollfd *fds[CONFIG_MXKBD_NPOLLWAITERS];

  /* Buffer used to collect and buffer incoming keyboard characters */
  uint16_t  headndx;      /* Buffer head index */
  uint16_t  tailndx;      /* Buffer tail index */
  uint8_t   kbdbuffer[CONFIG_MXKBD_BUFSIZE];
  uint8_t   crefs;        /* Reference count on the driver instance */
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

/* Driver methods. We export the keyboard as a standard character driver */

static int  mxkbd_open(FAR struct file *filep);
static int  mxkbd_close(FAR struct file *filep);
static ssize_t mxkbd_read(FAR struct file *filep,
                            FAR char *buffer, size_t len);
static ssize_t mxkbd_write(FAR struct file *filep,
                             FAR const char *buffer, size_t len);
static int  mxkbd_poll(FAR struct file *filep, FAR struct pollfd *fds,
                         bool setup);


#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static int mxkbd_interrupt_handler(FAR struct ioexpander_dev_s *ioe,
                                     ioe_pinset_t pinset, FAR void *arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_hidkbd_fops =
{
  mxkbd_open,             /* open */
  mxkbd_close,            /* close */
  mxkbd_read,             /* read */
  mxkbd_write,            /* write */
  NULL,                   /* seek */
  NULL,                   /* ioctl */
  mxkbd_poll              /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                  /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/


/****************************************************************************
 * Name: mxkbd_pollnotify
 ****************************************************************************/

static void mxkbd_pollnotify(FAR struct mxkbd_dev_s *priv)
{
  int i;

  for (i = 0; i < CONFIG_MXKBD_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = priv->fds[i];
      if (fds)
        {
          fds->revents |= (fds->events & POLLIN);
          if (fds->revents != 0)
            {
              uinfo("Report events: %02x\n", (unsigned int)fds->revents);
              nxsem_post(fds->sem);
            }
        }
    }
}

/****************************************************************************
 * Name: mxkbd_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int mxkbd_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct mxkbd_dev_s *priv;

  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Increment the reference count on the driver */

  priv->crefs++;

  return OK;
}

/****************************************************************************
 * Name: mxkbd_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int mxkbd_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct mxkbd_dev_s *priv;

  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Decrement the reference count on the driver */

  DEBUGASSERT(priv->crefs >= 1);

  priv->crefs--;

  return OK;
}

/****************************************************************************
 * Name: mxkbd_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t mxkbd_read(FAR struct file *filep, FAR char *buffer,
                            size_t len)
{
  FAR struct inode *inode;
  FAR struct mxkbd_dev_s *priv;
  size_t nbytes;
  uint16_t tail;
  int ret;

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

      if (++tail >= CONFIG_MXKBD_BUFSIZE)
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
 * Name: mxkbd_write
 *
 * Description:
 *   Standard character driver write method.
 *
 ****************************************************************************/

static ssize_t mxkbd_write(FAR struct file *filep, FAR const char *buffer,
                              size_t len)
{
  /* We won't try to write to the keyboard */

  return -ENOSYS;
}

/****************************************************************************
 * Name: mxkbd_poll
 *
 * Description:
 *   Standard character driver poll method.
 *
 ****************************************************************************/

static int mxkbd_poll(FAR struct file *filep, FAR struct pollfd *fds,
                         bool setup)
{
  FAR struct inode           *inode;
  FAR struct mxkbd_dev_s  *priv;
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

      for (i = 0; i < CONFIG_MXKBD_NPOLLWAITERS; i++)
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

      if (i >= CONFIG_MXKBD_NPOLLWAITERS)
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
          mxkbd_pollnotify(priv);
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
 * Name: mxkbd_putbuffer
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
#if 0
static void mxkbd_putbuffer(FAR struct mxkbd_dev_s *priv,
                               uint8_t keycode)
{
  uint16_t head;
  uint16_t tail;

  DEBUGASSERT(priv);

  /* Copy the next keyboard character into the user buffer. */

  head = priv->headndx;
  priv->kbdbuffer[head] = keycode;

  /* Increment the head index */

  if (++head >= CONFIG_MXKBD_BUFSIZE)
    {
      head = 0;
    }

  /* If the buffer is full, then increment the tail index to make space.
   * Drop old unread key presses.
   */

  tail = priv->tailndx;
  if (tail == head)
    {
      if (++tail >= CONFIG_MXKBD_BUFSIZE)
        {
          tail = 0;
        }

      /* Save the updated tail index */

      priv->tailndx = tail;
    }

  /* Save the updated head index */

  priv->headndx = head;
}
#endif
/****************************************************************************
 * Name: mxkbd_interrupt_handler
 *
 * Description:
 *   
 *
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static int mxkbd_interrupt_handler(FAR struct ioexpander_dev_s *ioe,
                                     ioe_pinset_t pinset, FAR void *arg)
{
  return OK;
}
#endif


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mxkbd_register
 *
 * Description:
 *   Configure the EdgeProMX Keyboard to use the provided IO Expander device
 *   instance. This will register the driver as /dev/kbdN where N is the
 *   minor device number
 *
 * Input Parameters:
 *   dev         - An IO Expander driver instance
 *   kbdminor    - The keyboard input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int mxkbd_register(FAR struct ioexpander_dev_s *dev, char kbdminor)
{
  FAR struct mxkbd_dev_s *priv;
  char kbddevname[DEV_NAMELEN];
  int ret;

  /* Debug Sanity Checks */

  DEBUGASSERT(dev != NULL);

  priv = (FAR struct mxkbd_dev_s *)kmm_zalloc(
    sizeof(struct mxkbd_dev_s));
  if (!priv)
    {
      ierr("ERROR: kmm_zalloc(%d) failed\n", sizeof(struct mxkbd_dev_s));
      return -ENOMEM;
    }

  /* Initialize the device driver instance */

  priv->dev       = dev;     /* Save the IO Expander device handle */
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

  /* Listen on COL0 ÷ COL7 */
  IOEP_ATTACH(priv->dev, 0xFF00, mxkbd_interrupt_handler, priv);

  IOEXP_SETDIRECTION(priv->dev, 0, IOEXPANDER_DIRECTION_OUT);
  IOEXP_SETDIRECTION(priv->dev, 1, IOEXPANDER_DIRECTION_OUT);
  IOEXP_SETDIRECTION(priv->dev, 2, IOEXPANDER_DIRECTION_OUT);
  IOEXP_SETDIRECTION(priv->dev, 3, IOEXPANDER_DIRECTION_OUT);
  IOEXP_SETDIRECTION(priv->dev, 4, IOEXPANDER_DIRECTION_OUT);
  IOEXP_SETDIRECTION(priv->dev, 5, IOEXPANDER_DIRECTION_OUT);
  IOEXP_SETDIRECTION(priv->dev, 6, IOEXPANDER_DIRECTION_OUT);
  IOEXP_SETDIRECTION(priv->dev, 7, IOEXPANDER_DIRECTION_OUT);

  IOEXP_SETDIRECTION(priv->dev, 8, IOEXPANDER_DIRECTION_IN_PULLUP);
  IOEXP_SETDIRECTION(priv->dev, 9, IOEXPANDER_DIRECTION_IN_PULLUP);
  IOEXP_SETDIRECTION(priv->dev, 10, IOEXPANDER_DIRECTION_IN_PULLUP);
  IOEXP_SETDIRECTION(priv->dev, 11, IOEXPANDER_DIRECTION_IN_PULLUP);
  IOEXP_SETDIRECTION(priv->dev, 12, IOEXPANDER_DIRECTION_IN_PULLUP);
  IOEXP_SETDIRECTION(priv->dev, 13, IOEXPANDER_DIRECTION_IN_PULLUP);
  IOEXP_SETDIRECTION(priv->dev, 14, IOEXPANDER_DIRECTION_IN_PULLUP);
  IOEXP_SETDIRECTION(priv->dev, 15, IOEXPANDER_DIRECTION_IN_PULLUP);

  snprintf(kbddevname, DEV_NAMELEN, DEV_FORMAT, kbdminor);
  iinfo("Registering %s\n", kbddevname);
  ret = register_driver(kbddevname, &g_hidkbd_fops, 0666, priv);
  if (ret != 0)
    {
      ierr("ERROR: Failed to register driver\n");
      goto errout_with_priv;
    }
  return OK;

errout_with_priv:
  nxsem_destroy(&priv->waitsem);
  nxsem_destroy(&priv->exclsem);
  kmm_free(priv);
  return ret;
}
