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
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/ioexpander/mcp23x17.h>

#ifdef CONFIG_MXKBD_ENCODED
#  include <nuttx/streams.h>
#  include <nuttx/input/kbd_codec.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This format is used to construct the /dev/kbd[n] device driver path. It is
 * defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT        "/dev/kbd%c"
#define DEV_NAMELEN       11

/* Keyboard matrix defines */

#define MATRIX_ROWS 8
#define MATRIX_COLS 8

#define LAYOUT( \
    k45, k44, k43, k42, k41, k40, k05, k04, k03, k02, k01, k00, \
    k55, k54, k53, k52, k51, k50, k15, k14, k13, k12, k11, k10, \
    k65, k64, k63, k62, k61, k60, k25, k24, k23, k22, k21, k20, \
    k75, k74, k73, k72, k71, k70, k35, k34, k33, k32, k31, k30, \
    k06, k16, k26, k36, k46, k56, k66, k76, \
    k07, k17, k27, k37, k47, k57, k67, k77) \
{ \
    { k00, k01, k02, k03, k04, k05, k06, k07 }, \
    { k10, k11, k12, k13, k14, k15, k16, k17 }, \
    { k20, k21, k22, k23, k24, k25, k26, k27 }, \
    { k30, k31, k32, k33, k34, k35, k36, k37 }, \
    { k40, k41, k42, k43, k44, k45, k46, k47 }, \
    { k50, k51, k52, k53, k54, k55, k56, k57 }, \
    { k60, k61, k62, k63, k64, k65, k66, k67 }, \
    { k70, k71, k72, k73, k74, k75, k76, k77 } \
}

#define NUMSCANCODES      104

#define MODIFER_LCTRL     (1 << 0) /* Left Ctrl */
#define MODIFER_LSHIFT    (1 << 1) /* Left Shift */
#define MODIFER_LALT      (1 << 2) /* Left Alt */
#define MODIFER_LGUI      (1 << 3) /* Left GUI */
#define MODIFER_RCTRL     (1 << 4) /* Right Ctrl */
#define MODIFER_RSHIFT    (1 << 5) /* Right Shift */
#define MODIFER_RALT      (1 << 6) /* Right Alt */
#define MODIFER_RGUI      (1 << 7) /* Right GUI */

#define S(kc)             (0x80 | (kc))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mxkbd_dev_s
{
  FAR struct ioexpander_dev_s *dev;

  mutex_t lock;           /* Exclusive access to dev */
  sem_t waitsem;          /* Signal waiting thread */
  volatile bool waiting;  /* Waiting for keyboard data */
  bool empty;             /* Keep track of data availability */

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

  uint8_t   layer;
  uint8_t   modifier;     /* Modifier keys */
  uint8_t   matrix[MATRIX_ROWS];
};

#ifdef CONFIG_MXKBD_ENCODED

/* The first and last scancode values with encode-able values */

#define FIRST_ENCODING    0x28          /* 0x28 Keyboard Return (ENTER) */
#define LAST_ENCODING     0x66          /* 0x66 Keyboard Power */

#define NUMENCODINGS (LAST_ENCODING - FIRST_ENCODING + 1)

static const uint8_t encoding[NUMENCODINGS] =
{
  /* 0x28-0x2f: Enter,escape,del,back-tab,space,_,+,{ */

  KEYCODE_ENTER,        0,
  KEYCODE_FWDDEL,       KEYCODE_BACKDEL,
  0,                    0,
  0,                    0,

  /* 0x30-0x37: },|,Non-US tilde,:,",grave tilde,<,> */

  0,                    0,
  0,                    0,
  0,                    0,
  0,                    0,

  /* 0x38-0x3f: /,CapsLock,F1,F2,F3,F4,F5,F6 */

  0,                    KEYCODE_CAPSLOCK,
  KEYCODE_F1,           KEYCODE_F2,
  KEYCODE_F3,           KEYCODE_F4,
  KEYCODE_F5,           KEYCODE_F6,

  /* 0x40-0x47: F7,F8,F9,F10,F11,F12,PrtScn,ScrollLock */

  KEYCODE_F7,           KEYCODE_F8,
  KEYCODE_F9,           KEYCODE_F10,
  KEYCODE_F11,          KEYCODE_F12,
  KEYCODE_PRTSCRN,      KEYCODE_SCROLLLOCK,

  /* 0x48-0x4f: Pause,Insert,Home,PageUp,
   * DeleteForward,End,PageDown,RightArrow
   */

  KEYCODE_PAUSE,        KEYCODE_INSERT,
  KEYCODE_HOME,         KEYCODE_PAGEUP,
  KEYCODE_FWDDEL,       KEYCODE_END,
  KEYCODE_PAGEDOWN,     KEYCODE_RIGHT,

  /* 0x50-0x57: LeftArrow,DownArrow,UpArrow,Num Lock,/,*,-,+ */

  KEYCODE_LEFT,         KEYCODE_DOWN,
  KEYCODE_UP,           KEYCODE_NUMLOCK,
  0,                    0,
  0,                    0,

  /* 0x58-0x5f: Enter,1-7 */

  KEYCODE_ENTER,        0,
  0,                    0,
  0,                    0,
  0,                    0,

  /* 0x60-0x66: 8-9,0,.,Non-US \,Application,Power */

  0,                    0,
  0,                    0,
  0,                    0,
  KEYCODE_POWER,
};
#endif

static const uint8_t ucmap[NUMSCANCODES] =
{
  0,    0,      0,      0,       'A',  'B',  'C',    'D',  /* 0x00-0x07: Reserved, errors, A-D */
  'E',  'F',    'G',    'H',     'I',  'J',  'K',    'L',  /* 0x08-0x0f: E-L */
  'M',  'N',    'O',    'P',     'Q',  'R',  'S',    'T',  /* 0x10-0x17: M-T */
  'U',  'V',    'W',    'X',     'Y',  'Z',  '!',    '@',  /* 0x18-0x1f: U-Z,!,@  */
  '#',  '$',    '%',    '^',     '&',  '*',  '(',    ')',  /* 0x20-0x27: #,$,%,^,&,*,(,) */
  '\n', '\033', '\177', 0,       ' ',  '_',  '+',    '{',  /* 0x28-0x2f: Enter,escape,del,back-tab,space,_,+,{ */
  '}',  '|',    0,      ':',     '"',  '~',  '<',    '>',  /* 0x30-0x37: },|,Non-US tilde,:,",grave tilde,<,> */
  '?',  0,       0,      0,      0,    0,    0,      0,    /* 0x38-0x3f: /,CapsLock,F1,F2,F3,F4,F5,F6 */
  0,    0,       0,      0,      0,    0,    0,      0,    /* 0x40-0x47: F7,F8,F9,F10,F11,F12,PrtScn,ScrollLock */
  0,    0,       0,      0,      0,    0,    0,      0,    /* 0x48-0x4f: Pause,Insert,Home,PageUp,DeleteForward,End,PageDown,RightArrow */
  0,    0,       0,      0,      '/',  '*',  '-',    '+',  /* 0x50-0x57: LeftArrow,DownArrow,UpArrow,Num Lock,/,*,-,+ */
  '\n', '1',     '2',    '3',    '4',  '5',  '6',    '7',  /* 0x58-0x5f: Enter,1-7 */
  '8',  '9',     '0',    '.',    0,    0,    0,      '=',  /* 0x60-0x67: 8-9,0,.,Non-US \,Application,Power,= */
};

static const uint8_t lcmap[NUMSCANCODES] =
{
  0,    0,       0,      0,      'a',  'b', 'c',     'd',  /* 0x00-0x07: Reserved, errors, a-d */
  'e',  'f',     'g',    'h',    'i',  'j', 'k',     'l',  /* 0x08-0x0f: e-l */
  'm',  'n',     'o',    'p',    'q',  'r', 's',     't',  /* 0x10-0x17: m-t */
  'u',  'v',     'w',    'x',    'y',  'z', '1',     '2',  /* 0x18-0x1f: u-z,1-2  */
  '3',  '4',     '5',    '6',    '7',  '8', '9',     '0',  /* 0x20-0x27: 3-9,0 */
  '\n', '\033',  '\177', '\t',   ' ',  '-', '=',     '[',  /* 0x28-0x2f: Enter,escape,del,tab,space,-,=,[ */
  ']',  '\\',    '\234', ';',    '\'', '`', ',',     '.',  /* 0x30-0x37: ],\,Non-US pound,;,',grave accent,,,. */
  '/',  0,       0,      0,      0,    0,   0,       0,    /* 0x38-0x3f: /,CapsLock,F1,F2,F3,F4,F5,F6 */
  0,    0,       0,      0,      0,    0,   0,       0,    /* 0x40-0x47: F7,F8,F9,F10,F11,F12,PrtScn,ScrollLock */
  0,    0,       0,      0,      0,    0,   0,       0,    /* 0x48-0x4f: Pause,Insert,Home,PageUp,DeleteForward,End,PageDown,RightArrow */
  0,    0,       0,      0,      '/',  '*', '-',     '+',  /* 0x50-0x57: LeftArrow,DownArrow,UpArrow,Num Lock,/,*,-,+ */
  '\n', '1',     '2',    '3',    '4',  '5', '6',     '7',  /* 0x58-0x5f: Enter,1-7 */
  '8',  '9',     '0',    '.',    0,    0,   0,       '=',  /* 0x60-0x67: 8-9,0,.,Non-US \,Application,Power,= */
};

enum keycode_e
{
  KC_NO = 0x00,
  KC_L1 = 0x01,
  KC_L2 = 0x02,
  KC_L3 = 0x03,
  KC_A = 0x04,
  KC_B = 0x05,
  KC_C = 0x06,
  KC_D = 0x07,
  KC_E = 0x08,
  KC_F = 0x09,
  KC_G = 0x0A,
  KC_H = 0x0B,
  KC_I = 0x0C,
  KC_J = 0x0D,
  KC_K = 0x0E,
  KC_L = 0x0F,
  KC_M = 0x10,
  KC_N = 0x11,
  KC_O = 0x12,
  KC_P = 0x13,
  KC_Q = 0x14,
  KC_R = 0x15,
  KC_S = 0x16,
  KC_T = 0x17,
  KC_U = 0x18,
  KC_V = 0x19,
  KC_W = 0x1a,
  KC_X = 0x1b,
  KC_Y = 0x1c,
  KC_Z = 0x1d,
  KC_1 = 0x1e,
  KC_2 = 0x1f,
  KC_3 = 0x20,
  KC_4 = 0x21,
  KC_5 = 0x22,
  KC_6 = 0x23,
  KC_7 = 0x24,
  KC_8 = 0x25,
  KC_9 = 0x26,
  KC_0 = 0x27,
  KC_ENTER = 0x28,
  KC_ESCAPE = 0x29,
  KC_BACKSPACE = 0x2a,
  KC_TAB = 0x2b,
  KC_SPACE = 0x2c,
  KC_MINUS = 0x2d,
  KC_EQUAL = 0x2e,
  KC_LEFT_BRACKET = 0x2f,
  KC_RIGHT_BRACKET = 0x30,
  KC_BACKSLASH = 0x31,
  KC_NONUS_HASH = 0x32,
  KC_SEMICOLON = 0x33,
  KC_QUOTE = 0x34,
  KC_GRAVE = 0x35,
  KC_COMMA = 0x36,
  KC_DOT = 0x37,
  KC_SLASH = 0x38,
  KC_CAPS_LOCK = 0x39,
  KC_F1 = 0x3a,
  KC_F2 = 0x3b,
  KC_F3 = 0x3c,
  KC_F4 = 0x3d,
  KC_F5 = 0x3e,
  KC_F6 = 0x3f,
  KC_F7 = 0x40,
  KC_F8 = 0x41,
  KC_F9 = 0x42,
  KC_F10 = 0x43,
  KC_F11 = 0x44,
  KC_F12 = 0x45,
  KC_PRINT_SCREEN = 0x46,
  KC_SCROLL_LOCK = 0x47,
  KC_PAUSE = 0x48,
  KC_INSERT = 0x49,
  KC_HOME = 0x4a,
  KC_PAGE_UP = 0x4b,
  KC_DELETE = 0x4c,
  KC_END = 0x4d,
  KC_PAGE_DOWN = 0x4e,
  KC_RIGHT = 0x4f,
  KC_LEFT = 0x50,
  KC_DOWN = 0x51,
  KC_UP = 0x52,
  KC_NUM_LOCK = 0x53,
  KC_KP_SLASH = 0x54,
  KC_KP_ASTERISK = 0x55,
  KC_KP_MINUS = 0x56,
  KC_KP_PLUS = 0x57,
  KC_KP_ENTER = 0x58,
  KC_KP_1 = 0x59,
  KC_KP_2 = 0x5a,
  KC_KP_3 = 0x5b,
  KC_KP_4 = 0x5c,
  KC_KP_5 = 0x5d,
  KC_KP_6 = 0x5e,
  KC_KP_7 = 0x5f,
  KC_KP_8 = 0x60,
  KC_KP_9 = 0x61,
  KC_KP_0 = 0x62,
  KC_KP_DOT = 0x63,
  KC_NONUS_BACKSLASH = 0x64,
  KC_APPLICATION = 0x65,
  KC_KB_POWER = 0x66,
  KC_LEFT_CTRL = 0x70,
  KC_LEFT_SHIFT = 0x71,
  KC_LEFT_ALT = 0x72,
  KC_LEFT_GUI = 0x73,
  KC_RIGHT_CTRL = 0x74,
  KC_RIGHT_SHIFT = 0x75,
  KC_RIGHT_ALT = 0x76,
  KC_RIGHT_GUI = 0x77,

  /* Alias */
  _______    = KC_NO,
  KC_TRNS    = KC_NO,
  KC_ENT     = KC_ENTER,
  KC_ESC     = KC_ESCAPE,
  KC_BSPC    = KC_BACKSPACE,
  KC_SPC     = KC_SPACE,
  KC_MINS    = KC_MINUS,
  KC_EQL     = KC_EQUAL,
  KC_LBRC    = KC_LEFT_BRACKET,
  KC_RBRC    = KC_RIGHT_BRACKET,
  KC_BSLS    = KC_BACKSLASH,
  KC_NUHS    = KC_NONUS_HASH,
  KC_SCLN    = KC_SEMICOLON,
  KC_QUOT    = KC_QUOTE,
  KC_GRV     = KC_GRAVE,
  KC_COMM    = KC_COMMA,
  KC_SLSH    = KC_SLASH,
  KC_CAPS    = KC_CAPS_LOCK,
  KC_PSCR    = KC_PRINT_SCREEN,
  KC_SCRL    = KC_SCROLL_LOCK,
  KC_BRMD    = KC_SCROLL_LOCK,
  KC_PAUS    = KC_PAUSE,
  KC_BRK     = KC_PAUSE,
  KC_BRMU    = KC_PAUSE,
  KC_INS     = KC_INSERT,
  KC_PGUP    = KC_PAGE_UP,
  KC_DEL     = KC_DELETE,
  KC_PGDN    = KC_PAGE_DOWN,
  KC_RGHT    = KC_RIGHT,
  KC_NUM     = KC_NUM_LOCK,
  KC_PSLS    = KC_KP_SLASH,
  KC_PAST    = KC_KP_ASTERISK,
  KC_PMNS    = KC_KP_MINUS,
  KC_PPLS    = KC_KP_PLUS,
  KC_PENT    = KC_KP_ENTER,
  KC_P1      = KC_KP_1,
  KC_P2      = KC_KP_2,
  KC_P3      = KC_KP_3,
  KC_P4      = KC_KP_4,
  KC_P5      = KC_KP_5,
  KC_P6      = KC_KP_6,
  KC_P7      = KC_KP_7,
  KC_P8      = KC_KP_8,
  KC_P9      = KC_KP_9,
  KC_P0      = KC_KP_0,
  KC_PDOT    = KC_KP_DOT,
  KC_NUBS    = KC_NONUS_BACKSLASH,
  KC_LCTL    = KC_LEFT_CTRL,
  KC_LSFT    = KC_LEFT_SHIFT,
  KC_LALT    = KC_LEFT_ALT,
  KC_LOPT    = KC_LEFT_ALT,
  KC_LGUI    = KC_LEFT_GUI,
  KC_LCMD    = KC_LEFT_GUI,
  KC_LWIN    = KC_LEFT_GUI,
  KC_RCTL    = KC_RIGHT_CTRL,
  KC_RSFT    = KC_RIGHT_SHIFT,
  KC_RALT    = KC_RIGHT_ALT,
  KC_ROPT    = KC_RIGHT_ALT,
  KC_ALGR    = KC_RIGHT_ALT,
  KC_RGUI    = KC_RIGHT_GUI,
  KC_RCMD    = KC_RIGHT_GUI,
  KC_RWIN    = KC_RIGHT_GUI,
  KC_TILD    = S(KC_GRAVE),
  KC_EXLM    = S(KC_1),
  KC_AT      = S(KC_2),
  KC_HASH    = S(KC_3),
  KC_DLR     = S(KC_4),
  KC_PERC    = S(KC_5),
  KC_CIRC    = S(KC_6),
  KC_AMPR    = S(KC_7),
  KC_ASTR    = S(KC_8),
  KC_LPRN    = S(KC_9),
  KC_RPRN    = S(KC_0),
  KC_UNDS    = S(KC_MINUS),
  KC_PLUS    = S(KC_EQUAL),
  KC_PIPE    = S(KC_BACKSLASH),
};


/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

static void mxkbd_putbuffer(FAR struct mxkbd_dev_s *priv,
                            uint8_t keycode);

/* Driver methods. We export the keyboard as a standard character driver */

static int  mxkbd_open(FAR struct file *filep);
static int  mxkbd_close(FAR struct file *filep);
static ssize_t mxkbd_read(FAR struct file *filep,
                          FAR char *buffer, size_t len);
static ssize_t mxkbd_write(FAR struct file *filep,
                           FAR const char *buffer, size_t len);
static int mxkbd_poll(FAR struct file *filep, FAR struct pollfd *fds,
                      bool setup);

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static int mxkbd_callback(FAR struct ioexpander_dev_s *ioe,
                          ioe_pinset_t pinset, FAR void *arg);
#else
#error Not supporting IOEXPANDER polling mode!
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
  NULL,                   /* truncate */
  NULL,                   /* mmap */
  mxkbd_poll              /* poll */
};

const uint8_t keymap[][MATRIX_ROWS][MATRIX_COLS] = {

  /* Qwerty
   * ,-----------------------------------------------------------------------------------.
   * | Tab  |   Q  |   W  |   E  |   R  |   T  |   Y  |   U  |   I  |   O  |   P  | Bksp |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * | Esc  |   A  |   S  |   D  |   F  |   G  |   H  |   J  |   K  |   L  |   ;  |  '   |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * | Shift|   Z  |   X  |   C  |   V  |   B  |   N  |   M  |   ,  |   .  |   /  |Enter |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * | Ctrl |      |      | Alt  |Raise |Space |Space |Lower | Left | Down |  Up  |Right |
   * `-----------------------------------------------------------------------------------'
   * ,-------------------------------------------------------.
   * |      |      |      |      |      |      |      |      |
   * |------+------+------+------+------+------+------+------|
   * |      |      |      |      |      |      |      |      |
   * `-------------------------------------------------------'
   */

  [0] = LAYOUT(
    KC_TAB,  KC_Q,    KC_W,    KC_E,    KC_R,      KC_T,   KC_Y,   KC_U,
      KC_I,    KC_O,    KC_P,    KC_BSPC,
    KC_ESC,  KC_A,    KC_S,    KC_D,    KC_F,      KC_G,   KC_H,   KC_J,
      KC_K,    KC_L,    KC_SCLN, KC_QUOT,
    KC_LSFT, KC_Z,    KC_X,    KC_C,    KC_V,      KC_B,   KC_N,   KC_M,
      KC_COMM, KC_DOT,  KC_SLSH, KC_ENT ,
    KC_LCTL, _______, _______, KC_LALT, KC_L2,   KC_SPC, KC_SPC,  KC_L1,
      KC_LEFT, KC_DOWN, KC_UP,   KC_RGHT,
    _______, _______, _______, _______, _______, _______, _______, _______,
    _______, _______, _______, _______, _______, _______, _______, _______
  ),

  /* Lower
   * ,-----------------------------------------------------------------------------------.
   * |   ~  |   !  |   @  |   #  |   $  |   %  |   ^  |   &  |   *  |   (  |   )  | Bksp |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * | Del  |  F1  |  F2  |  F3  |  F4  |  F5  |  F6  |   _  |   +  |   {  |   }  |  |   |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * |      |  F7  |  F8  |  F9  |  F10 |  F11 |  F12 |      |      |      | Home | End  |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * |      |      |      |      |      |      |      |      |      |      |      |      |
   * `-----------------------------------------------------------------------------------'
   * ,-------------------------------------------------------.
   * |      |      |      |      |      |      |      |      |
   * |------+------+------+------+------+------+------+------|
   * |      |      |      |      |      |      |      |      |
   * `-------------------------------------------------------'
   */

  [1] = LAYOUT(
    KC_TILD, KC_EXLM, KC_AT,   KC_HASH, KC_DLR,  KC_PERC, KC_CIRC, KC_AMPR,
      KC_ASTR,    KC_LPRN, KC_RPRN, KC_BSPC,
    KC_DEL,  KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_UNDS,
      KC_PLUS,    KC_LBRC, KC_RBRC, KC_PIPE,
    _______, KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,  KC_F12,  _______,
      _______, KC_HOME, KC_END,  _______,
    _______, _______, _______, _______, _______, _______, _______, _______,
      _______, _______, _______, _______,
    _______, _______, _______, _______, _______, _______, _______, _______,
    _______, _______, _______, _______, _______, _______, _______, _______
  ),

  /* Raise
   * ,-----------------------------------------------------------------------------------.
   * |   `  |   1  |   2  |   3  |   4  |   5  |   6  |   7  |   8  |   9  |   0  | Bksp |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * | Del  |  F1  |  F2  |  F3  |  F4  |  F5  |  F6  |   -  |   =  |   [  |   ]  |  \   |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * |      |  F7  |  F8  |  F9  |  F10 |  F11 |  F12 |      |      |Pg Up |Pg Dn |      |
   * |------+------+------+------+------+------+------+------+------+------+------+------|
   * |      |      |      |      |      |      |      |      |      |      |      |      |
   * `-----------------------------------------------------------------------------------'
   * ,-------------------------------------------------------.
   * |      |      |      |      |      |      |      |      |
   * |------+------+------+------+------+------+------+------|
   * |      |      |      |      |      |      |      |      |
   * `-------------------------------------------------------'
   */

  [2] = LAYOUT(
    KC_GRV,  KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,
      KC_8,    KC_9,    KC_0,    KC_BSPC,
    KC_DEL,  KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_MINS,
      KC_EQL,  KC_LBRC, KC_RBRC, KC_BSLS,
    _______, KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,  KC_F12,  _______,
      _______, KC_PGUP, KC_PGDN, _______,
    _______, _______, _______, _______, _______, _______, _______, _______,
      _______, _______, _______, _______,
    _______, _______, _______, _______, _______, _______, _______, _______,
    _______, _______, _______, _______, _______, _______, _______, _______
  )
};

#ifdef CONFIG_MXKBD_ENCODED
struct mxkbd_outstream_s
{
  struct lib_outstream_s stream;
  FAR struct mxkbd_dev_s *priv;
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mxkbd_putstream
 *
 * Description:
 *   A wrapper for mxkbd_putc that is compatible with the lib_outstream_s
 *   putc methods.
 *
 * Input Parameters:
 *   stream - The struct lib_outstream_s reference
 *   ch - The character to add to the user buffer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_MXKBD_ENCODED
static void mxkbd_putstream(FAR struct lib_outstream_s *stream, int ch)
{
  FAR struct mxkbd_outstream_s *privstream =
    (FAR struct mxkbd_outstream_s *)stream;

  DEBUGASSERT(privstream && privstream->priv);
  mxkbd_putbuffer(privstream->priv, (uint8_t)ch);
  stream->nput++;
}
#endif

/****************************************************************************
 * Name: mxkbd_mapscancode
 *
 * Description:
 *   Map a keyboard scancode to a printable ASCII character.  There is no
 *   support here for function keys or cursor controls in this version of
 *   the driver.
 *
 * Input Parameters:
 *   scancode - Scan code to be mapped.
 *   modifier - Ctrl,Alt,Shift,GUI modifier bits
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline uint8_t mxkbd_mapscancode(uint8_t scancode, uint8_t modifier)
{
  /* Range check */

  if (scancode >= NUMSCANCODES)
    {
      return 0;
    }

  if (scancode & 0x80)
    {
      modifier |= MODIFER_LSHIFT;
      scancode &= 0x7f;
    }

  /* Is either shift key pressed? */

  if ((modifier & (MODIFER_LSHIFT | MODIFER_RSHIFT)) != 0)
    {
      return ucmap[scancode];
    }
  else
    {
      return lcmap[scancode];
    }
}

/****************************************************************************
 * Name: mxkbd_encodescancode
 *
 * Description:
 *  Check if the key has a special function encoding and, if it does, add
 *  the encoded value to the user buffer.
 *
 * Input Parameters:
 *   priv - Driver internal state
 *   scancode - Scan code to be mapped.
 *   modifier - Ctrl, Alt, Shift, GUI modifier bits
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_MXKBD_ENCODED
static inline void mxkbd_encodescancode(FAR struct mxkbd_dev_s *priv,
                                        uint8_t scancode, uint8_t modifier)
{
  uint8_t encoded;

  /* Check if the raw scancode is in a valid range */

  if (scancode >= FIRST_ENCODING && scancode <= LAST_ENCODING)
    {
      /* Yes the value is within range */

      encoded = encoding[scancode - FIRST_ENCODING];
      
      if (encoded)
        {
          struct mxkbd_outstream_s mxkbdstream;

          /* And it does correspond to a special function key */

          mxkbdstream.stream.putc = mxkbd_putstream;
          mxkbdstream.stream.nput = 0;
          mxkbdstream.priv        = priv;

          /* Add the special function value to the user buffer */

          kbd_specpress((enum kbd_keycode_e)encoded,
                        (FAR struct lib_outstream_s *)&mxkbdstream);
        }
    }
}
#endif

/****************************************************************************
 * Name: mxkbd_extract_key
 *
 * Description:
 *  Check if the key has a special function encoding and, if it does, add
 *  the encoded value to the user buffer.
 *
 * Input Parameters:
 *   priv - Driver internal state
 *   key - key code
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void mxkbd_extract_key(FAR struct mxkbd_dev_s *priv, uint8_t key)
{
  uint8_t keycode;
  int ret;
  bool newstate;

  /* Add the newly detected keystrokes to our internal buffer */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return;
    }

  if (key != KC_NO)
    {
      /* Yes.. Add it to the buffer. */

      /* Map the keyboard scancode to a printable ASCII
       * character.  There is no support here for function keys
       * or cursor controls in this version of the driver.
       */

      keycode = mxkbd_mapscancode(key, priv->modifier);

      /* Zero at this point means that the key does not map to a
       * printable character.
       */

      if (keycode != 0)
        {
          /* Handle control characters. Zero after this means
           * a valid, NUL character.
           */

          if ((priv->modifier & (MODIFER_LCTRL | MODIFER_RCTRL)) != 0)
            {
              keycode &= 0x1f;
            }

          /* Copy the next keyboard character into the user
           * buffer.
           */

          mxkbd_putbuffer(priv, keycode);
        }

      /* The zero might, however, map to a special keyboard
       * action (such as a cursor movement or function key).
       * Attempt to encode the special key.
       */

#ifdef CONFIG_MXKBD_ENCODED
      else
        {
          mxkbd_encodescancode(priv, key, priv->modifier);
        }
#endif
    }

  /* Is there data available? */

  newstate = (priv->headndx == priv->tailndx);
  if (!newstate)
    {
      /* Did we just transition from no data available to data
       * available?  If so, wake up any threads waiting for the
       * POLLIN event.
       */

      if (priv->empty)
        {
          poll_notify(priv->fds, CONFIG_MXKBD_NPOLLWAITERS, POLLIN);
        }

      /* Yes.. Is there a thread waiting for keyboard data now? */

      if (priv->waiting)
        {
          /* Yes.. wake it up */

          nxsem_post(&priv->waitsem);
          priv->waiting = false;
        }
    }

  priv->empty = newstate;
  nxmutex_unlock(&priv->lock);
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

  ret = nxmutex_lock(&priv->lock);
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
          nxmutex_unlock(&priv->lock);
          ret = nxsem_wait_uninterruptible(&priv->waitsem);
          if (ret < 0)
            {
              return ret;
            }

          ret = nxmutex_lock(&priv->lock);
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
  nxmutex_unlock(&priv->lock);
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
  ret = nxmutex_lock(&priv->lock);
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
          poll_notify(priv->fds, CONFIG_MXKBD_NPOLLWAITERS, POLLIN);
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
  nxmutex_unlock(&priv->lock);
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

/****************************************************************************
 * Name: mxkbd_select_row
 *
 * Description:
 *   Selecting single row on the keyboard.
 *   The keyboard matrix has MATRIX_ROWS rows and MATRIX_COLS coloumns.
 *
 * Input Parameters:
 *   priv - Driver internal state
 *   row - Row index
 *
 * Returned Value:
 *   TRUE is returned on success. Otherwise FALSE.
 *
 ****************************************************************************/
static bool mxkbd_select_row(FAR struct mxkbd_dev_s *priv, uint8_t row)
{
  /* MATRIX_ROWS rows, index starting with 0 */

  if (row >= MATRIX_ROWS)
    {
      return FALSE;
    }

  IOEXP_WRITEPIN(priv->dev, row, FALSE);

  return TRUE;
}

/****************************************************************************
 * Name: mxkbd_select_rows
 *
 * Description:
 *   Selecting all rows on the keyboard.
 *
 * Input Parameters:
 *   priv - Driver internal state
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
static void mxkbd_select_rows(FAR struct mxkbd_dev_s *priv)
{
  uint8_t i;

  for (i = 0; i < MATRIX_ROWS; i++)
    {
      mxkbd_select_row(priv, i);
    }
}

/****************************************************************************
 * Name: mxkbd_unselect_row
 *
 * Description:
 *   Unselecting single row on the keyboard.
 *   The keyboard matrix has MATRIX_ROWS rows and MATRIX_COLS coloumns.
 *
 * Input Parameters:
 *   priv - Driver internal state
 *   row - Row index
 *
 * Returned Value:
 *   TRUE is returned on success. Otherwise FALSE.
 *
 ****************************************************************************/
static bool mxkbd_unselect_row(FAR struct mxkbd_dev_s *priv, uint8_t row)
{
  /* 8 rows, index starting with 0 */

  if (row >= MATRIX_ROWS)
    {
      return FALSE;
    }

  IOEXP_WRITEPIN(priv->dev, row, TRUE);

  return TRUE;
}

/****************************************************************************
 * Name: mxkbd_unselect_rows
 *
 * Description:
 *   Unselecting all rows on the keyboard.
 *
 * Input Parameters:
 *   priv - Driver internal state
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
static void mxkbd_unselect_rows(FAR struct mxkbd_dev_s *priv)
{
  uint8_t i;

  for (i = 0; i < MATRIX_ROWS; i++)
    {
      mxkbd_unselect_row(priv, i);
    }
}

/****************************************************************************
 * Name: mxkbd_read_cols_on_row
 *
 * Description:
 *   Reading coloumns on sigle row
 *
 * Input Parameters:
 *   priv - Driver internal state
 *   row - Row index
 *
 * Returned Value:
 *   row value
 *
 ****************************************************************************/
static uint8_t mxkbd_read_cols_on_row(FAR struct mxkbd_dev_s *priv, uint8_t row)
{
  uint8_t i;
  uint8_t row_shifter = 1;
  uint8_t row_value = 0;

  if (!mxkbd_select_row(priv, row))
    {
      return 0xff;
    }

  /* MATRIX_COLS coloumns, index starting with MATRIX_ROWS */

  for (i = MATRIX_ROWS; i < (MATRIX_ROWS + MATRIX_COLS);
       i++, row_shifter <<= 1)
    {
      int ret;
      bool pin_state = FALSE;

      ret = IOEXP_READPIN(priv->dev, i, &pin_state);
      if (ret < 0)
        {
          continue;
        }

      /* Check if key pressed */

      if (!pin_state)
        {
          uint8_t keycode = keymap[0][row][i - MATRIX_ROWS];

          if (keycode > 0 && keycode < 4)
            {
              /* Got momentary layer keycode */

              if ((keycode & 0x03) > priv->layer)
                {
                  priv->layer = keycode & 0x03;
                }
            }
          else if (keycode >= KC_LEFT_CTRL && keycode <= KC_RIGHT_GUI)
            {
              /* Got modifier keycode */

              priv->modifier |= 1 << (keycode & 0x07);
            }
        }

      row_value |= pin_state ? 0 : row_shifter;
    }


  mxkbd_unselect_row(priv, row);

  return row_value;
}

/****************************************************************************
 * Name: mxkbd_callback
 *
 * Description:
 *
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static int mxkbd_callback(FAR struct ioexpander_dev_s *dev,
                          ioe_pinset_t pinset, FAR void *arg)
{
  struct mxkbd_dev_s *priv = (struct mxkbd_dev_s *)arg;
  uint8_t row;
  uint8_t col = 0;
  uint8_t matrix[MATRIX_ROWS];

  /* Reset layer and modifier. They will be scanned soon. */

  priv->layer = 0;
  priv->modifier = 0;

  mxkbd_unselect_rows(priv);

  for (row = 0; row < MATRIX_ROWS; row++)
    {
      matrix[row] = mxkbd_read_cols_on_row(priv, row);
    }

  mxkbd_select_rows(priv);

  /* Check for keystrokes and add them to keyboard buffer */

  for (row = 0; row < MATRIX_ROWS; row++)
    {
      uint8_t change = priv->matrix[row] ^ matrix[row];
      
      while (change)
        {
          if (change & 0x01)
            {
              if (!(matrix[row] & (1 << col)))
                {
                  uint8_t layer = priv->layer;

                  do
                    {
                      if (keymap[layer][row][col] != KC_TRNS)
                        {
                          mxkbd_extract_key(priv, keymap[layer][row][col]);
                          break;
                        }

                      layer = layer > 0 ? layer - 1 : 0;
                    }
                  while (layer);
                } 
            }

          change >>= 1;
          col++;
        }

      /* Update matrix */

      priv->matrix[row] = matrix[row];
    }

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

  nxmutex_init(&priv->lock);   /* Initialize device mutex */
  nxsem_init(&priv->waitsem, 0, 0);

  priv->empty     = true;

  /* Listen on COL0 ÷ COL7 */

  IOEP_ATTACH(priv->dev, 0xff00, mxkbd_callback, priv);

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

  mxkbd_callback(priv->dev, 0xff00, priv);

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
  nxmutex_destroy(&priv->lock);
  nxsem_destroy(&priv->waitsem);
  kmm_free(priv);
  return ret;
}
