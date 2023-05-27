/*
ADAFRUIT RETROGAME UTILITY: remaps buttons on Raspberry Pi GPIO header to
virtual USB keyboard presses.  Great for classic game emulators!  Retrogame
is interrupt-driven and efficient (typically < 0.3% CPU use, even w/heavy
button-mashing) and debounces inputs for glitch-free gaming.

****** IF YOU ARE SEARCHING FOR THE ioStandard[] OR ioTFT[] TABLES: ******
GPIO pin and key mapping is now set in a configuration file; there's no
fixed table to edit in this code (as in earlier releases).  An example
config file is provided in retrogame.cfg.  By default, retrogame looks for
this file in /boot, but an alternate (full pathname) can be passed as a
command line argument.

Connect one side of button(s) to GND pin (there are several on the GPIO
header, but see later notes) and the other side to GPIO pin of interest.
Internal pullups are used; no resistors required.  MCP23017 I2C port
expanders are also supported (up to 8).  Pin mapping is:

    0 -  31   GPIO header 'P5' (Broadcom pin numbers)
   32 -  47   MCP23017 at address 0x20
   48 -  63   MCP23017 at address 0x21
   64 -  79   MCP23017 at address 0x22
   80 -  95   MCP23017 at address 0x23
   96 - 111   MCP23017 at address 0x24
  112 - 127   MCP23017 at address 0x25
  128 - 143   MCP23017 at address 0x26 *** Arcade Bonnet default address
  144 - 159   MCP23017 at address 0x27 *** Arcade Bonnet alt address

Config file IRQ command must be used to bind a GPIO pin to an I2C address!

Must be run as root, i.e. 'sudo ./retrogame &' or edit /etc/rc.local to
launch automatically at system startup.

Early Raspberry Pi Linux distributions might not have the uinput kernel
module installed by default.  To enable this, add a line to /etc/modules:

    uinput

This code has bloated into a stinking seven-headed hydra.
Please no more feature creep.

Written by Phil Burgess for Adafruit Industries, distributed under BSD
License.  Adafruit invests time and resources providing this open source
code, please support Adafruit and open-source hardware by purchasing
products from Adafruit!

Copyright (c) 2013, 2016 Adafruit Industries.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

--------------------------------------------------------------------------------

This is a fork to additionally implement directly connected keyboard and two
joysticks as an input matrix with a fixed scheme. At present this is not
configurable in retrogame.cfg.

Changes made:
- Used constants to make the most of all hardware limits slightly less magic.
- Placed and altered code taken from Github project
  https://github.com/dabonetn/C64USBKey/blob/master/C64Key3-TeensyLC/C64Key3-TeensyLC.ino
- Invoked several more file descriptors to directly transmit keys and joystick
  movements to be used by the interrupt handler of this project.
- Unfortunately the input matrix requires constant output switching for strobe
  input reading. The interrupt handler's timeout had to be used for this fast
  scanning resulting in much higher cpu load.
- Thus the variable "timeout" is now sort of a flag to divert into the right
  keyboard handling part within the main loop if the matrix mode is used.

*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>
#include <stdbool.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <dirent.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/signalfd.h>
#include <sys/inotify.h>
#include <linux/input.h>
#include <linux/uinput.h>
#include <linux/i2c-dev.h>
#include <bcm_host.h>
#include "keyTable.h"

// -------------------------- BEGIN vH -------------------------------------
// #define NULL (uint16_t) 0
#define HIGH true
#define LOW  false
#define OUTPUT true
#define INPUT  false
#define INPUT_PULLUP false  // @@@@@vH to be changed

#define MAXKEYMAP  80       // number of keys on matrix keymap
#define MAXHYBRID   7
#define MAXHYBRID2  5
#define MAXKEYCOMB 20
#define MAXROWS     8       // number of rows and cols to form the matrix of
#define MAXCOLS    10       // GPIOs

#define MAXDWORD    6       // Number of words for all 160 IO plus the
                            // above special flags
// Invoke number of words for GPIO ones plus the special word plus matrix words:
#define MAXHWDWORD  (MAXDWORD + (int) (MAXKEYMAP / 32 + 1))       
#define INTMATRIX   6       // Last DWord within "intstate" for virtual imputs
// @@@@@vH Fake data:
// int RowPinMap[MAXROWS] = {27, 22};  // Convert Row number to associated output pin
// int ColPinMap[MAXCOLS] = {23, 24};  // Convert Col number to associated input pin
int RowPinMap[MAXROWS] = {26,  9, 11,  5,  6, 13, 19, 10};  // Convert Row number to associated output pin
int ColPinMap[MAXCOLS] = {21, 20, 16, 23,  8, 25, 24,  7, 27, 22};  // Convert Col number to associated input pin

uint16_t keyMap[MAXKEYMAP]={
// Joystick Keys are the indicated by JoyNumber(Direction), i.e Joy1UP is up on joystick one.
// Firebuttons are Noted JoyF(Joystick number)- Fire Button number, i.e Joy2F1 Joystick 2, fire button 1, Joy2F2 Joystick 2, fire button 2.
// Current values for the Joysticks are located at the end of each line. KP stands for numeric keypad inputs.

//  Del        Return       LR         F7         F1               F3                F5          UD            Null      Null     
//  3          W            A          4          Z                S                 E           LSHFT         Joy2Down  Joy1Down 
//  5          R            D          6          C                F                 T           X             Joy2Left  Joy1Left 
//  7          Y            G          8          B                H                 U           V             Joy2Right Joy1Right
//  9          I            J          Zero       M                K                 O           N             Joy2F1    Joy1F1   
//  +          P            L          -          .                :                 @           ,             Joy2F2    Joy1F2   
//  Pound      *            ;          Home       RSHFT            =                 Pi          /             Restore   Null     
//  1          LEFT ARROW   CTRL       2          SPC              C=                Q           RunStop       Joy2Up    Joy1Up   
          
KEY_BACKSPACE, KEY_ENTER,   KEY_RIGHT, KEY_F7,    KEY_F1,          KEY_F3,           KEY_F5,     KEY_DOWN,     0,        0,              
KEY_3,         KEY_W,       KEY_A,     KEY_4,     KEY_Z,           KEY_S,            KEY_E,     KEY_LEFTSHIFT, KEY_KP2,  KEY_KPASTERISK, 
KEY_5,         KEY_R,       KEY_D,     KEY_6,     KEY_C,           KEY_F,            KEY_T,      KEY_X,        KEY_KP4,  KEY_KPMINUS,    
KEY_7,         KEY_Y,       KEY_G,     KEY_8,     KEY_B,           KEY_H,            KEY_U,      KEY_V,        KEY_KP6,  KEY_KPPLUS,     
KEY_9,         KEY_I,       KEY_J,     KEY_0,     KEY_M,           KEY_K,            KEY_O,      KEY_N,        KEY_KP5,  KEY_SCROLLLOCK, 
KEY_MINUS,     KEY_P,       KEY_L,     KEY_EQUAL, KEY_DOT,       KEY_SEMICOLON,  KEY_LEFTBRACE,  KEY_COMMA,    KEY_KP3,  KEY_KP1,        
KEY_INSERT, KEY_RIGHTBRACE, KEY_APOSTROPHE, KEY_HOME, KEY_RIGHTSHIFT, KEY_BACKSLASH, KEY_DELETE, KEY_SLASH,    KEY_PAGEUP, 0,              
KEY_1,         KEY_GRAVE,   KEY_TAB,   KEY_2,     KEY_SPACE,       KEY_LEFTCTRL,     KEY_Q,      KEY_ESC,      KEY_KP8,  KEY_KPSLASH,    

/* Please be aware that in C64Key3-TeensyLC.ino the key F10 was emitted instead
of Page Up to send Commodore's Restore key into Vice emulator or whereever in-
side your Raspi. Further I replaced the left Alt key from C64Key3-TeensyLC.ino
with the Esc key to act as Run Stop. In C64Key3-TeensyLC.ino this Esc key was
emitted for Commodore's left arrow on the upper left of the keyboard which here
is replaceably sending the grave key. This all seems to be what is default in
Retropie/Vice anyways, as are all the keys being emitted as joystick actions. */
};         
           
uint16_t Hybridkeys[MAXHYBRID] = {
           // Hybrid Keys. These are the shifted values.  
           // These allow keys to be passed in windows mode that are not shifted values of the original key. 
           // This allows the windows mode to use the cursor keys and F1-F8. The cursor keys and F keys are default, The restore key is F10.
KEY_LEFT, KEY_F8, KEY_F2, KEY_F4, KEY_F6, KEY_UP, KEY_F12,   // LR F8 F2 F4 F6 UD Restore
};

uint16_t Hybridkeys2[MAXHYBRID2] = {
           // Hybrid Keys2. These are the shifted values.  Defaults to the CTRL Key
           // These allow keys to be passed in windows mode that are not shifted values of the original key. 
           // Page Down, F11, `, Page UP, F9 Mapped to F7,F1,F3,F8 and Restore
KEY_PAGEDOWN, KEY_F11, KEY_GRAVE, KEY_PAGEUP, KEY_F9,
};

// This replaces "hybrid keys" of C64Key3-TeensyLC.ino and makes it configurable
struct keycombinations        // "hold" and "press" are positions within matrix
{                             // of the C64 keyboard.
  int      hold;              // emulate is the key from keyTable.h to be sent
  int      press;             // to the Raspi as a keystroke.
  uint16_t emulate;
} keykomb[MAXKEYCOMB] = {
// hold press emulate
  { 77,  4,  KEY_Q  },         // Esc-F1 --> Q
  { 77,  5,  KEY_BACKSLASH  }, // Esc-F3 --> Backslash
  { 17,  2,  KEY_LEFT  },      // Shift-Right --> Left
  { 64,  2,  KEY_LEFT  },      // Shift-Right --> Left
  { 17,  3,  KEY_F8  },        // Shift-F7 --> F8
  { 64,  3,  KEY_F8  },        // Shift-F7 --> F8
  { 17,  4,  KEY_F2  },        // Shift-F1 --> F2
  { 64,  4,  KEY_F2  },        // Shift-F1 --> F2
  { 17,  5,  KEY_F4  },        // Shift-F3 --> F4
  { 64,  5,  KEY_F4  },        // Shift-F3 --> F4
  { 17,  6,  KEY_F6  },        // Shift-F5 --> F6
  { 64,  6,  KEY_F6  },        // Shift-F5 --> F6
  { 17,  7,  KEY_UP  },        // Shift-Down --> Up
  { 64,  7,  KEY_UP  },        // Shift-Down --> Up
  { 17, 68,  KEY_F12  },       // Shift-Restore --> F12
  { 72,  3,  KEY_PAGEDOWN  },  // Ctrl-F7 --> PageDown
  { 72,  4,  KEY_F11  },       // Ctrl-F1 --> F11
  { 72,  5,  KEY_GRAVE  },     // Ctrl-F3 --> §
  { 72,  6,  KEY_PAGEUP  },    // Ctrl-F5 --> PageUp
  { 72, 68,  KEY_F9  }         // Ctrl-Restore --> F9
}; /* This array has to be full because also "0" is a valid key code. For un-
      needed lines use -1 as the "hold" key in the first column.              */

int keyPos=0;
int keyDown[MAXKEYMAP];
// long lastDebounceTime[MAXKEYMAP];
// int debounceDelay=50;
// int shift=0;
int HybridKeyboard=1;           // Select 0 for normal or 1 for the left HybridKey allowing all f keys and cursor keys in windows mode. (Also has a shifted restore key)
int HybridKey=17;               // Position of the Hybrid Select key in the keymap, Left Shift = 17, Right Shift = 64
int HybridKey2=72;              // Position of the Hybrid2 Select key in the keymap, CTRL = 72
// -------------------------- END   vH -------------------------------------

// Global variables and such -----------------------------------------------

bool
   running      = true,              // Signal handler will set false (exit)
   isEarlyPi    = false;             // true=Pi1Rev1, false=all other
extern char
  *__progname,                       // Program name (for error reporting)
  *program_invocation_name;          // Full name as invoked (path, etc.)
char
   sysfs_root[] = "/sys/class/gpio", // Location of Sysfs GPIO files
  *cfgPath,                          // Directory containing config file
  *cfgName      = NULL,              // Name (no path) of config
  *cfgPathname,                      // Full path/name to config file
   debug        = 0,                 // 0=off, 1=cfg file, 2=live buttons
   startupDebug = 0,                 // Initial debug level before cfg load
   readAddr     = 0x10;              // For MCP23017 reads (INTCAPA reg addr)
int
   key[MAXHWDWORD * 32 + 1],         // Keycodes assigned to GPIO pins, also
                                     // virtual ones above 160
   fileWatch,                        // inotify file descriptor
   keyfd1       = -1,                // /dev/uinput file descriptor
   keyfd2       = -1,                // /dev/input/eventX file descriptor
   keyfd        = -1,                // = (keyfd2 >= 0) ? keyfd2 : keyfd1;
   i2cfd[8],                         // /dev/i2c-1 MCP23017 file descriptors
   vulcanTime   = 1500,              // Pinch time in milliseconds
   debounceTime = 20,                // 20 ms for button debouncing
   repTime1     = 700,               // Key hold time to begin repeat
   repTime2     = 100;               // Time between key repetitions
   // Note: auto-repeat is for navigating the game-selection menu using the
   // 'gamera' utility; MAME disregards key repeat events (as it should).
uint32_t
   intstate[MAXHWDWORD],             // Button last-read state (bitmask)
   extstate[MAXHWDWORD],             // Button debounced state
   vulcanMask[MAXHWDWORD],           // Bitmask of 'Vulcan nerve pinch' keys
   mcpMask      = 0;                 // Bitmask of GPIOs assigned to MCP IRQs
uint8_t
   mcpI2C[32];                       // GPIO index to MCP23017 I2C addr
volatile unsigned int
  *gpio         = NULL;              // GPIO register table
struct pollfd
   p[35],                            // File descriptors for poll()
   pm[35];                           // File descriptors for matrix

enum commandNum {
  CMD_NONE, // Used during config file read (no command ID'd yet)
  CMD_KEY,  // Key-to-GPIO mapping command
  CMD_IRQ,  // MCP23017 IRQ pin & address assignment
  CMD_GND,  // Pin-to-ground assignment
  CMD_DEBUG // Set debug level
};

// dict of config file commands that AREN'T keys (KEY_*)
dict command[] = { // Struct is defined in keyTable.h
  { "GND"     , CMD_GND   },
  { "GROUND"  , CMD_GND   },
  { "IRQ"     , CMD_IRQ   },
  { "DEBUG"   , CMD_DEBUG },
  // Might add commands here for fine-tuning debounce & repeat settings
  {  NULL     , -1        } }; // END-OF-LIST

#define GPIO_BASE              0x200000
#define BLOCK_SIZE             (4*1024)
#define GPPUD                  (0x94 / 4)
#define GPPUDCLK0              (0x98 / 4)
#define PULLUPDN_OFFSET_2711_0 57
#define PULLUPDN_OFFSET_2711_1 58
#define PULLUPDN_OFFSET_2711_2 59
#define PULLUPDN_OFFSET_2711_3 60

#define IODIRA                 0x00
#define IOCONA                 0x0A

#define GND                    KEY_CNT

// Debug levels: 0 = off, 1 = config file errors, 2 = + config file status,
// 3 = + report button states 'live'.


// Some utility functions --------------------------------------------------

// Detect Pi board type.  Not detailed, just enough for GPIO compatibilty:
// true  = Pi 1 Model B revision 1 (some GPIO pin numbers were different)
// false = All other board types
static bool earlyPiDetect(void) {
  FILE *fp;
  char  buf[1024], *ptr;
  int   n;
  bool  isEarly = false; // Assume "modern" Pi by default

  // Relies on info in /proc/cmdline.  If this becomes unreliable
  // in the future, alt code below uses /proc/cpuinfo if any better.
#if 1
  if((fp = fopen("/proc/cmdline", "r"))) {
    while(fgets(buf, sizeof(buf), fp)) {
      if((ptr = strstr(buf, "boardrev=")) &&
                (sscanf(&ptr[9], "%x", &n) == 1) &&
                ((n == 0x02) || (n == 0x03))) {
        isEarly = true; // Appears to be an early Pi
        break;
      }
    }
    fclose(fp);
  }
#else
  char s[8];
  if((fp = fopen("/proc/cpuinfo", "r"))) {
    while(fgets(buf, sizeof(buf), fp)) {
      if((ptr = strstr(buf, "Revision")) &&
                (sscanf(&ptr[8], " : %x", &n) == 1) &&
                ((n == 0x02) || (n == 0x03))) {
        isEarly = true; // Appears to be an early Pi
        break;
      }
    }
    fclose(fp);
  }
#endif

  return isEarly;
}

// Set one GPIO pin attribute through the Sysfs interface.
static int pinSetup(int pin, char *attr, char *value) {
  char filename[50];
  int  fd, w, len = strlen(value);
  sprintf(filename, "%s/gpio%d/%s", sysfs_root, pin, attr);
  if((fd = open(filename, O_WRONLY)) < 0) return -1;
  w = write(fd, value, len);
  close(fd);
  return (w != len); // 0 = success
}

// Configure GPIO internal pull up/down/none
static void pull(int bitmask, int state) {
  if(gpio[PULLUPDN_OFFSET_2711_3] != 0x6770696f) {
    // Pi 4 insights from RPi.GPIO:
    unsigned int pull = state ? (3 - state) : state;
    for(int bit=0; bit<32; bit++) {
      if(!(bitmask & (1UL << bit))) continue;
      int pullreg = PULLUPDN_OFFSET_2711_0 + (bit >> 4);
      int pullshift = (bit & 0xF) << 1;
      unsigned int pullbits;
      pullbits      = gpio[pullreg];
      pullbits     &= ~(3 << pullshift);
      pullbits     |= (pull << pullshift);
      gpio[pullreg] = pullbits;
    }
  } else {
    // Legacy pull up/down method:
    volatile unsigned char shortWait;
    gpio[GPPUD]     = state;         // 2=up, 1=down, 0=none
    for(shortWait=150;--shortWait;); // Min 150 cycle wait
    gpio[GPPUDCLK0] = bitmask;       // Set pullup mask
    for(shortWait=150;--shortWait;); // Wait again
    gpio[GPPUD]     = 0;             // Reset pullup registers
    gpio[GPPUDCLK0] = 0;
  }
}

// Restore GPIO and uinput to startup state; un-export any Sysfs pins used,
// don't leave any filesystem cruft; restore any GND pins to inputs and
// disable previously-set pull-ups.  Write errors are ignored as pins may be
// in a partially-initialized state.
static void pinConfigUnload() {
  char buf[50];
  int  fd, i;

  if(debug >= 2) printf("%s: Unloading config\n", __progname);

  // Close GPIO file descriptors
  for(i=0; i<32; i++) {
    if(p[i].fd >= 0) {
      close(p[i].fd);
      p[i].fd = -1;
    }
    p[i].events = p[i].revents = 0;
  }

  // Close uinput file descriptors
  keyfd = -1;
  if(keyfd2 >= 0) {
    close(keyfd2);
    keyfd2 = -1;
  }
  if(keyfd1 >= 0) {
    ioctl(keyfd1, UI_DEV_DESTROY);
    close(keyfd1);
    keyfd1 = -1;
  }

  // Un-export GPIO pins (0-31)
  sprintf(buf, "%s/unexport", sysfs_root);
  if((fd = open(buf, O_WRONLY)) >= 0) {
    for(i=0; i<32; i++) {
      // Restore GND items to inputs
      if(key[i] >= GND) pinSetup(i, "direction", "in");
      // And un-export all items regardless
      sprintf(buf, "%d", i);
      write(fd, buf, strlen(buf));
    }
    close(fd);
  }

  // Disable GPIO pullups (0-31)
  uint32_t mask = vulcanMask[0] | mcpMask;
  for(i=0; i<32; i++) {
    if((key[i] > KEY_RESERVED) && (key[i] < GND))
      mask |= (1 << i);
  }
  pull(mask, 0); // Disable pullups

  // Do some GPIO dis-configuration for any MCO23017(s).
  // GNDs are set back to inputs; other config (pullups, etc.)
  // are currently left in whatever state.
  for(i=0; i<8; i++) {
    if(i2cfd[i] > 0) {
      // Determine which bits were previously set GND
      uint8_t  j;
      uint16_t gndMask = 0;
      for(j=0; j<16; j++) { // 16 bits per MCP
        if(key[32 + i * 16 + j] == GND)
          gndMask |= (1 << j);
      }
      // Read chip config
      uint8_t cfg[3];
      cfg[0] = IODIRA;
      write(i2cfd[i], cfg, 1);
      read(i2cfd[i], &cfg[1], sizeof(cfg) - 1);
      // Change IODIRA,B GND bits back to inputs
      cfg[1] = (cfg[1] |  gndMask      );
      cfg[2] = (cfg[2] | (gndMask >> 8));
      // Write to chip, close device
      write(i2cfd[i], cfg, sizeof(cfg));
      close(i2cfd[i]);
      i2cfd[i] = 0;
    }
  }

  // Reset pin-and-key-related globals
  for(i=0; i<161; i++) key[i] = KEY_RESERVED;
  memset(intstate  , 0, sizeof(intstate));
  memset(extstate  , 0, sizeof(extstate));
  memset(vulcanMask, 0, sizeof(vulcanMask));
  memset(mcpI2C    , 0, sizeof(mcpI2C));
  memset(i2cfd     , 0, sizeof(i2cfd));
  mcpMask = 0;
}

// Quick-n-dirty error reporter; print message, clean up and exit.
static void err(char *msg) {
  printf("%s: %s.  Try 'sudo %s' ?\n", __progname, msg,
    program_invocation_name);
  if(gpio) pinConfigUnload();
  exit(1);
}

// Filter function for scandir(), identifies possible device candidates for
// simulated keypress events (distinct from actual USB keyboard(s)).
static int filter1(const struct dirent *d) {
  if(!strncmp(d->d_name, "input", 5)) { // Name usu. 'input' + #
    // Read contents of 'name' file inside this subdirectory,
    // if it matches the retrogame executable, that's probably
    // the device we want...
    char  filename[300], line[100];
    FILE *fp;
    sprintf(filename, "/sys/devices/virtual/input/%s/name",
      d->d_name);
    memset(line, 0, sizeof(line));
    if((fp = fopen(filename, "r"))) {
      fgets(line, sizeof(line), fp);
      fclose(fp);
    }
    if(!strncmp(line, __progname, strlen(__progname))) return 1;
  }
  return 0;
}

// A second scandir() filter, checks for filename of 'event' + #
static int filter2(const struct dirent *d) {
  return !strncmp(d->d_name, "event", 5);
}

// Search for name in dictionary, return assigned value (-1 = not found)
static int dictSearch(char *str, dict *d) {
  int i;
  for(i=0; d[i].name && strcasecmp(str, d[i].name); i++);
  return d[i].value;
}

// If this is a "Revision 1" Pi board (no mounting holes), remap certain
// pin numbers for compatibility.  Can then use 'modern' pin numbers
// regardless of board type.
static int pinRemap(int i) {
  if(isEarlyPi) {
    if     (i ==  2) return  0;
    else if(i ==  3) return  1;
    else if(i == 27) return 21;
  }
  return i;
}

// Config file handlage ----------------------------------------------------

// Load pin/key configuration from cfgPathname.
static void pinConfigLoad() {

  // Config file format is super simple, just per-line keyword and
  // argument(s) with whitespace delimiters...can parse it ourselves
  // here.  Configuration libraries such as libconfig, libconfuse
  // have some potent features but enforce a correspondingly more
  // exacting syntax on the user; do not want if we can avoid it.

  FILE            *fp;
  char             buf[50];
  enum commandNum  cmd = CMD_NONE;
  int              stringLen      = 0,
                   wordCount      = 0,
                   keyCode        = KEY_RESERVED,
                   i, c, k, fd, bitmask, dLevel = -1,
                   mcpPin = -1, mcpAddr = -1;
  bool             readingString  = false,
                   isComment      = false;
  uint32_t         pinMask[MAXHWDWORD];

  if(debug >= 2) printf("%s: Loading config\n", __progname);

  // Read config file into key[] table -------------------------------

  if(NULL == (fp = fopen(cfgPathname, "r"))) {
    if(debug >= 1) printf("%s: could not open config file '%s' "
      "(not fatal, continuing)\n", __progname, cfgPathname);
    return; // Not fatal; file might be created later
  }

  do { // Deep nesting, please excuse shift to two-space indents...
    c = getc(fp);
    if(isspace(c) || (c <= 0)) { // If whitespace char...
      if(readingString && !isComment) {
        // Switching from string-reading to whitespace-skipping.
        // Cap off & process current string, reset readingString flag.
        buf[stringLen] = 0;
        if(wordCount == 1) {
          // First word on line.  Search key dict, then command dict
          memset(pinMask, 0, sizeof(pinMask));
          keyCode = KEY_RESERVED;
          if((k = dictSearch(buf, keyTable)) >= 0) {
            // Start of key command
            cmd     = CMD_KEY;
            keyCode = k;
          } else if((k = dictSearch(buf, command)) >= 0) {
            // Not a key, is other command (e.g. GND, DEBUG)
            cmd = k;
          } else if(debug >= 1) {
            printf("%s: unknown key or command '%s' (not fatal, "
              "continuing)\n", __progname, buf);
          }
        } else {
          // Word #2+ on line; Certain commands may accumulate
          // values (e.g. keys w/pinch).
          char *endptr;
          int   arg = strtol(buf, &endptr, 0);
          switch(cmd) {
           case CMD_KEY:
           case CMD_GND:
            if((*endptr) || (arg < 0) || (arg > 159)) {
              // Non-NUL character indicates not full string
              // was parsed, i.e. bad numeric input.
              if(debug >= 1) {
                printf("%s: invalid pin '%s' (not fatal, "
            "continuing)\n", __progname, buf);
              }
            } else {
              // Add pin # (in 'arg') to list
              arg = pinRemap(arg); // Handle early Pi boards
              pinMask[arg/32] |= (1 << (arg&31));
            }
            break;
           case CMD_IRQ:
            switch(wordCount) {
             case 2: // word 2 = GPIO pin number for IRQ, MUST be 0-31
              if((*endptr) || (arg < 0) || (arg > 31)) {
                if(debug >= 1) {
                  printf("%s: invalid pin '%s' (not fatal, "
              "continuing)\n", __progname, buf);
                }
              } else {
                mcpPin = pinRemap(arg); // Handle early Pi boards
              }
              break;
             case 3: // word 3 = I2C addr, must be 0-7 or 0x20-0x27
              if((*endptr) || (arg < 0) || (arg > 0x27) ||
                ((arg > 7) && (arg < 0x20))) {
                if(debug >= 1) {
                  printf("%s: invalid I2C address '%s' (not fatal, "
              "continuing)\n", __progname, buf);
                }
              } else {
                mcpAddr = (arg < 0x20) ? (arg + 0x20) : arg;
              }
              break;
             default:
              if(debug >= 1) {
                printf("%s: extraneous parameter '%s' (not fatal, "
            "continuing)\n", __progname, buf);
              }
              break;
            }
            break;
           case CMD_DEBUG:
            if((*endptr) || (arg < 0) || (arg > 3)) {
              if(debug >= 1) {
                printf("%s: invalid debug level '%s' "
                  "(not fatal, continuing)\n", __progname, buf);
              }
            } else {
              dLevel = arg;
            }
            break;
           default:
            break;
          }
        }
        readingString = false;
      }
      if((c == '\n') || (c <= 0)) { // If EOF or EOL
        // Execute last line if useful command
        switch(cmd) {
         case CMD_KEY:
          // Count number of pins on line (k)
          for(k=i=0; i<160; i++) {
            if(pinMask[i/32] & (1 << (i&31))) {
              k++;
              // Un-assign any pins previously assigned GND.
              if(key[i] == GND) key[i] = KEY_RESERVED;
            }
          }
          if(k == 1) { // Key assigned to one pin
            for(i=0; !(pinMask[i/32] & (1<<(i&31))); i++); // Find bit
            key[i] = keyCode;
            if(debug >= 2) {
              printf("%s: virtual key %d assigned to GPIO%02d\n",
                __progname, keyCode, i);
            }
          } else if(k > 1) {
            memcpy(vulcanMask, pinMask, sizeof(pinMask));
            key[160] = keyCode;
            if(debug >= 2) {
              printf("%s: virtual key %d has GPIO bitmask "
                "%04X%04X%04X%04X%04X\n", __progname, key[160],
                vulcanMask[4], vulcanMask[3], vulcanMask[2],
                vulcanMask[1], vulcanMask[0]);
            }
          }
          break;
         case CMD_IRQ:
          if((mcpPin >= 0) && (mcpAddr >= 0)) { // Got all params?
            if(debug >= 2) {
              printf("%s: MCP23017 on GPIO%02d, I2C address 0x%02X\n",
                __progname, mcpPin, mcpAddr);
            }
            mcpI2C[mcpPin] = mcpAddr; // GPIO pin # to I2C address
            mcpMask |= (1 << mcpPin);
            mcpPin = mcpAddr = -1;
          }
          break;
         case CMD_GND:
          // One or more GND pins
          for(i=0; i<160; i++) {
            if(pinMask[i/32] & (1 << (i&31))) {
              key[i] = GND;
              if(debug >= 2) {
                printf("%s: GPIO%02d assigned GND\n", __progname, i);
              }
            }
          }
          // Clear any vulcanMask bits that are now GNDs
          for(i=k=0; i<5; i++) {
            vulcanMask[i] &= ~pinMask[i];
            if(vulcanMask[i]) k = 1;
          }
          if(!k) key[160] = KEY_RESERVED; // All vulcan bits clobbered
          break;
         case CMD_DEBUG:
          if(debug || (dLevel > 0)) {
            printf("%s: debug level %d\n", __progname, dLevel);
          }
          debug = dLevel;
          break;
         default:
          break;
        }
        // Reset ALL string-reading flags
        readingString = false;
        stringLen     = 0;
        wordCount     = 0;
        isComment     = false;
        cmd           = CMD_NONE;
      }
    } else {                        // Non-whitespace char
      if(isComment) continue;
      if(!readingString) {
        // Switching from whitespace-skipping
        // to string-reading.  Reset string.
        readingString = true;
        stringLen     = 0;
        // Is it beginning of a comment?
        // If so, ignore chars to next EOL.
        if(c == '#') {
          isComment = true;
          continue;
        }
        wordCount++;
      }
      // Append characer to current string
      if(stringLen < (sizeof(buf) - 1)) buf[stringLen++] = c;
    }
  } while(c > 0);

  fclose(fp);

  // If debug was previously set (either in file or by starting
  // in foreground) but line is now deleted, set debug level to
  // whatever its startup case was.
  if(debug && (dLevel < 0)) {
    debug = startupDebug;
    printf("%s: debug level %d\n", __progname, debug);
  }

  // Set up GPIO -----------------------------------------------------
// @@@@@vH Maybe consider vulcanmask?
  bitmask = vulcanMask[0] | mcpMask;
  for(i=0; i<32; i++) {
    if((key[i] > KEY_RESERVED) && (key[i] < GND))
      bitmask |= (1 << i);
  }
  pull(bitmask, 2); // Enable pullups on input pins
  for(i=0; (i<5) && !vulcanMask[i]; i++); // If no vulcanMask bits,
  if(i >= 5) key[160] = KEY_RESERVED;     // make sure no vulcanKey
  // Pullups on MCP23017 devices will be a separate pass later

  // All other GPIO config is handled through the sysfs interface.

  sprintf(buf, "%s/export", sysfs_root);
  if((fd = open(buf, O_WRONLY)) < 0) // Open Sysfs export file
    err("Can't open GPIO export file");
  intstate[0] = 0;
  for(i=0; i<32; i++) {
    if((key[i] == KEY_RESERVED) && !(bitmask & (1<<i)))
      continue;
    sprintf(buf, "%d", i);
    write(fd, buf, strlen(buf));    // Export pin
    pinSetup(i, "active_low", "0"); // Don't invert
    if(key[i] >= GND) {
      // Set pin to output, value 0 (ground)
      if(pinSetup(i, "direction", "out") ||
         pinSetup(i, "value"    , "0"))
        err("Pin config failed (GND)");
    } else {
      // Set pin to input, detect edge events
      char x;
      // Plain GPIOs: detect both RISING and FALLING
      // edges.  Port expanders: detect FALLING only.
      if(pinSetup(i, "direction", "in") ||
         pinSetup(i, "edge",
          (mcpMask & (1<<i)) ? "falling" : "both")) {
        err("Pin config failed");
      }
      // Get initial pin value.  This is for plain GPIOs
      // only; MCP23017 will be a separate pass later.
      sprintf(buf, "%s/gpio%d/value", sysfs_root, i);
      if((p[i].fd = open(buf, O_RDONLY | O_NONBLOCK)) < 0) {
        sprintf(buf, "Can't access pin value %d", i);
        err(buf);
      }
      if((read(p[i].fd, &x, 1) == 1) && (x == '0'))
        intstate[0] |= 1 << i;
      p[i].events  = POLLPRI | POLLERR | POLLHUP | POLLNVAL;
      p[i].revents = 0;
    }
  }
  close(fd); // Done w/Sysfs exporting

  // Set up uinput

  // Attempt to create uidev virtual keyboard
  if((keyfd1 = open("/dev/uinput", O_WRONLY | O_NONBLOCK)) >= 0) {
    (void)ioctl(keyfd1, UI_SET_EVBIT, EV_KEY);
    for(i=0; i<161; i++) {
      if((key[i] >= KEY_RESERVED) && (key[i] < GND))
        (void)ioctl(keyfd1, UI_SET_KEYBIT, key[i]);
    }
// -------------------------- BEGIN vH -------------------------------------
    for(i=0; i < MAXKEYMAP; i++) {
      if(keyMap[i] > 0)
        (void)ioctl(keyfd1, UI_SET_KEYBIT, keyMap[i]);
    }
/*
    for(i=0; i < MAXHYBRID; i++) {
      if(Hybridkeys[i] > 0)
        (void)ioctl(keyfd1, UI_SET_KEYBIT, Hybridkeys[i]);
    }
    for(i=0; i < MAXHYBRID2; i++) {
      if(Hybridkeys2[i] > 0)
        (void)ioctl(keyfd1, UI_SET_KEYBIT, Hybridkeys2[i]);
    }  */
    for(i=0; i < MAXKEYCOMB; i++) {
      if(keykomb[i].emulate > 0)
        (void)ioctl(keyfd1, UI_SET_KEYBIT, keykomb[i].emulate);
    }
// -------------------------- END   vH -------------------------------------
    struct uinput_user_dev uidev;
    memset(&uidev, 0, sizeof(uidev));
    snprintf(uidev.name, UINPUT_MAX_NAME_SIZE, "retrogame");
    uidev.id.bustype = BUS_USB;
    uidev.id.vendor  = 0x1;
    uidev.id.product = 0x1;
    uidev.id.version = 1;
    if(write(keyfd1, &uidev, sizeof(uidev)) < 0)
      err("write failed");
    if(ioctl(keyfd1, UI_DEV_CREATE) < 0)
      err("DEV_CREATE failed");
    if(debug >= 3) printf("%s: uidev init OK\n", __progname);
  }

  // SDL2 (used by some newer emulators) wants /dev/input/eventX
  // instead -- BUT -- this only exists if there's a physical USB
  // keyboard attached or if the above code has run and created a
  // virtual keyboard.  On older systems this method doesn't apply,
  // events can be sent to the keyfd1 virtual keyboard above...so,
  // this code looks for an eventX device and (if present) will use
  // that as the destination for events, else fallback on keyfd1.

  // The 'X' in eventX is a unique identifier (typically a numeric
  // digit or two) for each input device, dynamically assigned as
  // USB input devices are plugged in or disconnected (or when the
  // above code runs, creating a virtual keyboard).  As it's
  // dynamically assigned, we can't rely on a fixed number -- it
  // will vary if there's a keyboard connected at startup.

  struct dirent **namelist;
  int             n;
  char            evName[300] = "";

  if((n = scandir("/sys/devices/virtual/input",
    &namelist, filter1, NULL)) > 0) {
    // Got a list of device(s).  In theory there should
    // be only one that makes it through the filter (name
    // matches retrogame)...if there's multiples, only
    // the first is used.  (namelist can then be freed)
    char path[300];
    sprintf(path, "/sys/devices/virtual/input/%s",
      namelist[0]->d_name);
    for(i=0; i<n; i++) free(namelist[i]);
    free(namelist);
    // Within the given device path should be a subpath with
    // the name 'eventX' (X varies), again theoretically
    // should be only one, first in list is used.
    if((n = scandir(path, &namelist, filter2, NULL)) > 0) {
      sprintf(evName, "/dev/input/%s",
        namelist[0]->d_name);
      for(i=0; i<n; i++) free(namelist[i]);
      free(namelist);
    }
  }

  if(!evName[0]) { // Nothing found?  Use fallback method...
    // Kinda lazy skim for last item in /dev/input/event*
    // This is NOT guaranteed to be retrogame, but if the
    // above method fails for some reason, this may be
    // adequate.  If there's a USB keyboard attached at
    // boot, it usually instantiates in /dev/input before
    // retrogame, so even if it's then removed, the index
    // assigned to retrogame stays put...thus the last
    // index mmmmight be what we need.
    struct stat st;
    for(i=99; i>=0; i--) {
      sprintf(buf, "/dev/input/event%d", i);
      if(!stat(buf, &st)) break; // last valid device
    }
    strcpy(evName, (i >= 0) ? buf : "/dev/input/event0");
  }

  keyfd2 = open(evName, O_WRONLY | O_NONBLOCK);
  keyfd  = (keyfd2 >= 0) ? keyfd2 : keyfd1;
  // keyfd1 and 2 are global and are held open (as a destination for
  // key events) until pinConfigUnload() is called.
  if((debug >= 3) && keyfd2) printf("%s: SDL2 init OK\n", __progname);

  // Configure MCP23017 port expander(s)

  uint8_t  cfg1[] = { 0x05  , 0x00 }, // If bank 1, switch to 0
           cfg2[] = { IOCONA, 0x44 }, // Bank 0, INTB=A, seq, OD IRQ
           cfg3[23];                  // Read-modify-write chip cfg
  uint16_t inputMask, gndMask;

  if(mcpMask) { // Any port expanders mentioned in config?
    for(i=0; i<8; i++) { // 8 possible MCP23017 indices
      uint8_t j;
      inputMask = gndMask = 0; // Bitmasks of keys, gnds on this device
      for(j=0; j<16; j++) { // 16 bits per MCP
        k = key[32 + i * 16 + j];
        if(k == GND)              gndMask   |= (1 << j);
        else if(k > KEY_RESERVED) inputMask |= (1 << j);
      }

      if(inputMask || gndMask) { // Referenced in config?
        // Each MCP23017 is assigned a separate file descriptor;
        // each bonded once to a specific I2C address (via ioctl)
        // so that the ioctl isn't required for every transaction.
        if((i2cfd[i] = open("/dev/i2c-1", O_RDWR | O_NONBLOCK)) > 0) {
          ioctl(i2cfd[i], I2C_SLAVE, 0x20 + i);
          // Configure chip as we need it (sequential addr, etc.).
          // This does mean any other application also using the
          // chip might be clobbered if it uses a different config.
          write(i2cfd[i], cfg1, sizeof(cfg1));
          write(i2cfd[i], cfg2, sizeof(cfg2));
          // Some bits are preserved as best we can...read
          // registers, change bits for retrogame, write back.
          // This is done in two passes; first one does some
          // polarity stuff, second pass sets more and reads state.
          cfg3[0] = IODIRA;
          write(i2cfd[i], cfg3, 1);
          read(i2cfd[i], &cfg3[1], 4); // Read partial config
          // Change IODIRA,B bits for inputs & GNDs (leave others)
          cfg3[1] = (cfg3[1] |  inputMask      ) &  ~gndMask;
          cfg3[2] = (cfg3[2] | (inputMask >> 8)) & ~(gndMask >> 8);
          // Set IPOLA,B for inputs+GNDs (polarity matches input logic)
          cfg3[3] &= ~( inputMask | gndMask);
          cfg3[4] &= ~((inputMask | gndMask) >> 8);
          write(i2cfd[i], cfg3, 5); // Write partial config
          write(i2cfd[i], cfg3, 1); // Next read is from IODIRA
          read(i2cfd[i], &cfg3[1], sizeof(cfg3) - 1); // Read full cfg
          // Enable interrupts on input pins (GPINTENA,B)
          cfg3[5] |= inputMask;
          cfg3[6] |= inputMask >> 8;
          // Skip DEFVALA,B
          cfg3[9] = cfg3[10] = 0; // INTCONA,B: compre prev pin value
          // Skip IOCON (x2)
          // Set GPPUA,B bits on input pins
          cfg3[13] |= inputMask;
          cfg3[14] |= inputMask >> 8;
          // Skip INTFA,B, INTCAPA,B, read GPIOA,B into int/extstate[]
          int      idx = 1 + i / 2; // Index (1-4) into int/extstate[]
          uint8_t  bit;             // Bit to read in GPIOA/B
          uint32_t abit, bbit;      // Bit to set in int/ext state
          if(i & 1) {               // In upper half of int/ext state
            abit = 0x00800000;
            bbit = 0x80000000;
          } else {                  // In lower half
            abit = 0x00000080;
            bbit = 0x00008000;
          }
          for(bit=0x80; bit; bit >>= 1, abit >>= 1, bbit >>= 1) {
            // Invert logic; set bits in intstate[] are buttons
            // pressed, while set bits in config are pulled up.
            if(cfg3[19] & bit) intstate[idx] &= ~abit;
            else               intstate[idx] |=  abit;
            if(cfg3[20] & bit) intstate[idx] &= ~bbit;
            else               intstate[idx] |=  bbit;
          }
          // Clear OLATA,B bits on GND outputs
          cfg3[21] &=  ~gndMask;
          cfg3[22] &= ~(gndMask >> 8);
          write(i2cfd[i], cfg3, sizeof(cfg3));
          // Clear interrupt by reading GPIOA/B+INTCAPA/B
          write(i2cfd[i], &readAddr, 1);
          read(i2cfd[i], cfg3, 4);
        }
      }
    }
  }

  memcpy(extstate, intstate, sizeof(extstate));
}

// -------------------------- BEGIN vH -------------------------------------

// Have a nice Arduino like encapsulation:
void pinMode(int pin, int mode)
{
  int fd;
  char buf[50];
  int bitmask=0;

  char s[55];

  sprintf(buf, "%s/export", sysfs_root);
  if((fd = open(buf, O_WRONLY)) < 0) // Open Sysfs export file
      err("Can't open GPIO export file");
//  intstate[0] = 0;

  sprintf(buf, "%d", pin);
  write(fd, buf, strlen(buf));    // Export pin
  pinSetup(pin, "active_low", "0"); // Don't invert

  sprintf(s, "Pin %d config failed (direction=%s)", pin, 
        (mode==OUTPUT ? "out" : "in"));
  if(pinSetup(pin, "direction", (mode==OUTPUT ? "out" : "in")))
    err(s);
  if (mode==INPUT_PULLUP)
  {
    if(pinSetup(pin, "edge", "both"))
      err("Pin config failed");
    bitmask |= (1 << (pin));
    pull(bitmask, 2); // Enable pullups on input pins
//    pull(bitmask, 0); // Disable pullups on output pins
    if(debug >= 3) printf("%s: matrix output bitmask %X\n", __progname, bitmask);
  }
  close(fd); // Done w/Sysfs exporting
}
void digitalWrite(int pin, int state)
{
  if(pinSetup(pin, "value"    , (state==LOW ? "0" : "1")))
    err("Pin write failed (GND)");
  if(debug >= 3) printf("%s: digitalWrite pin %d state to '%s'\n",
                        __progname, pin, (state==LOW ? "0" : "1"));
}
int digitalRead(int pin)
{
  char c;            // Pin input value ('0'/'1')

  c = 0; // By default, don't issue SYN event
  // Read current pin state, store in internal state flag,
  // flag, but don't issue to uinput yet -- must debounce!
  lseek(pm[pin].fd, 0, SEEK_SET);
  read(pm[pin].fd, &c, 1);
//  if(c == '0')      intstate[0] |=  (1 << i);
//  else if(c == '1') intstate[0] &= ~(1 << i);

  if(debug >= 3) printf("%s: digitalRead pin %d state is '%c'\n", __progname, pin, c);
  
  return (c == '0' ? LOW : HIGH);
}

// Taken from C64Key3-TeensyLC.ino

void setup()
{
  char buf[50];

  for (int i = 0; i < MAXROWS * MAXCOLS; i++) keyDown[i]=0; // Set all keys as up
  
  for (int Row=0; Row<MAXROWS; Row++) {
    pinMode(RowPinMap[Row], INPUT_PULLUP); 
    close(p[RowPinMap[Row]].fd);  // dirty, close interrupt file handling
  }

  for (int Col=0; Col<MAXCOLS; Col++) {
    pinMode(ColPinMap[Col], INPUT_PULLUP); // use internal pullups to hold pins high

    sprintf(buf, "%s/gpio%d/value", sysfs_root, ColPinMap[Col]);
    if((pm[ColPinMap[Col]].fd = open(buf, O_RDONLY | O_NONBLOCK)) < 0) {
      sprintf(buf, "digitalRead: Can't access pin value %d", ColPinMap[Col]);
      err(buf);
    }
    close(p[ColPinMap[Col]].fd);  // dirty, close interrupt file handling
  }

  for (int i = MAXDWORD; i < MAXHWDWORD; i++) intstate[i] = 0;
}

int loop() // main keyboard scanning loop
{
  int InputValue; // Replaces digitalread, so that global variable can be removed
  int RowPin;     // Replaces rowPinSet, so that global variable can be removed
  int change=false; // return value for interrupt handler

  uint8_t  a=MAXDWORD;   // to count within intstate[] above regular GPIO words
  uint32_t b=1;   // to shift bits
  for (int Row=0; Row<MAXROWS; Row++) // scan through all rows
  {
    RowPin = RowPinMap[Row];  // Map logical row to output pin

    pinMode(RowPin,OUTPUT);   // Set output pin to OUTPUT
    usleep(2000);                // Delay to make sure it has time to go HIGH before switching to INPUT
    digitalWrite(RowPin,LOW); // Set output pin to LOW

    for (int Col=0; Col<MAXCOLS; Col++) // scan through columns
    {
      keyPos=Col+(Row*MAXCOLS); // calculate character map position
      
      InputValue = digitalRead(ColPinMap[Col]); // LOW = Key pressed, HIGH = Key Not Pressed

      if (keyMap[keyPos]>0) // debounce is in main()
      {
        if ((InputValue == LOW) && keyDown[keyPos]==0) // if a key is pressed and wasn't already down
        {
          keyDown[keyPos]=keyMap[keyPos];        // put the right character in the keydown array
          if (HybridKeyboard==1)
          {
            for (int i = 0; i < MAXKEYCOMB; i++) // for all combos
            {
              if (keykomb[i].hold >= 0 &&
                  keyDown[keykomb[i].hold] && keyDown[keykomb[i].press])
              {
//                printf("%s: C64 komb %d key %d is DOWN hold=%d press=%d\n",
//                           __progname, i, keyPos, keykomb[i].hold, keykomb[i].press);
//                keyDown[keykomb[i].hold] = 0; // race condition
	        intstate[MAXDWORD + keykomb[i].hold / 32] &= ~(1 << (keykomb[i].hold % 32)); // reset position of key
// printf("%s: KEYCLEAR bit=%d word=%d intstate[]=%d\n", __progname, (1 << (keykomb[i].hold % 32)), MAXDWORD + keykomb[i].hold / 32, intstate[MAXDWORD + keykomb[i].hold / 32]);
                keyDown[keyPos] = keykomb[i].emulate;
              }
            }
          }
          // Keys to be emitted cannot be preset at startup because of the spe-
          // cial combo mappings in the lines just above this.
          key[MAXDWORD*32+keyPos] = keyDown[keyPos]; // this key being used
          intstate[a] |=  b; // (1 << (Col));  // set keystate for virtual GPIO
// printf("%s: KEYDOWN bit=%d key=%d word=%d intstate[a]=%d\n", __progname, b, key[Col+192], a, intstate[a]);
          change = true;
          
          if(debug) printf("%s: C64 matrix pos %d key %d is DOWN a=%d b=%d\n",
                           __progname, keyPos, keyMap[keyPos], a, b);
        }
        if ((InputValue == HIGH) && keyDown[keyPos]!=0) // key is up and a character is stored in the keydown position
        {
          keyDown[keyPos]=0; // set keydown array position as up
          intstate[a] &= ~b; // ~(1 << (Col));  // release keystate for virtual GPIO
          change = true;

          if(debug) printf("%s: C64 matrix pos %d key %d is   UP a=%d b=%d\n",
                           __progname, keyPos, keyMap[keyPos], a, b);
        }
      }
      
      b <<= 1;
      if (!b)  { a++; b = 1; }  // overflow handling towards next word
    }
    digitalWrite(RowPin,HIGH);    // Set output pin to HIGH
    usleep(2000);                // Delay to make sure it has time to go HIGH before switching to INPUT
    pinMode(RowPin,INPUT_PULLUP); // Set output pin back to INPUT with pullup to make sure it stays HIGH
  }
  return change;
}
// -------------------------- END   vH -------------------------------------

// Handle signal events (i=32), config file change events (33) or
// config directory contents change events (34).
static void pollHandler(int i) {

  if(i == 32) { // Signal event
    struct signalfd_siginfo info;
    read(p[i].fd, &info, sizeof(info));
    if(info.ssi_signo == SIGHUP) { // kill -1 = force reload
      if(debug >= 2) {
        printf("%s: SIGHUP received; force config "
          "reload\n", __progname);
      }
      pinConfigUnload();
      pinConfigLoad();
      setup();
    } else { // Other signal = abort program
      running = false;
    }
  } else { // Change in config file or directory contents
    char evBuf[1000];
    //int  evCount = 0;
    int  bufPos = 0,
         bytesRead = read(p[i].fd, evBuf, sizeof(evBuf));
    while(bufPos < bytesRead) {
      struct inotify_event *ev =
        (struct inotify_event *)&evBuf[bufPos];

      //printf("EVENT %d:\n", evCount++);
      //printf("\tinotify event mask: %08x\n", ev->mask);
      //printf("\tlen: %d\n", ev->len);
      //if(ev->len > 0)
        //printf("\tname: '%s'\n", ev->name);

      if(ev->mask & IN_MODIFY) {
        if(debug >= 2) {
          printf("%s: Config file changed\n",
            __progname);
        }
        pinConfigUnload();
        pinConfigLoad();
        setup();
      } else if(ev->mask & IN_IGNORED) {
        // Config file deleted -- stop watching it
        if(debug >= 2) {
          printf("%s: Config file removed\n",
            __progname);
        }
        inotify_rm_watch(p[1].fd, fileWatch);
        // Closing the descriptor turns out to be
        // important, as removing the watch itself
        // creates another IN_IGNORED event.
        // Avoids turtles all the way down.
        close(p[33].fd);
        p[33].fd     = -1;
        p[33].events =  0;
        // Pin config is NOT unloaded...
        // keep using prior values for now.
      } else if(ev->mask & IN_MOVED_FROM) {
        // File moved/renamed from directory...
        // check if it's the one we're monitoring.
        if(!strcmp(ev->name, cfgName)) {
          // It's our file -- stop watching it
          if(debug >= 2) {
            printf("%s: Config file "
              "moved out\n", __progname);
          }
          inotify_rm_watch(p[33].fd, fileWatch);
          close(p[33].fd);
          p[33].fd     = -1;
          p[33].events =  0;
          // Pin config is NOT unloaded...
          // keep using prior values for now.
        } else {
          // Some other file -- disregard
        }
      } else if(ev->mask & (IN_CREATE | IN_MOVED_TO)) {
        // File moved/renamed to directory...
        // check if it's the one we're monitoring for.
        if(!strcmp(ev->name, cfgName)) {
          // It's our file -- start watching it!
          if(debug >= 2) {
            printf("%s: Config file "
              "moved in\n", __progname);
          }
          if(p[33].fd >= 0) { // Existing file?
            inotify_rm_watch(
              p[33].fd, fileWatch);
            close(p[33].fd);
          }
          p[33].fd   = inotify_init();
          fileWatch = inotify_add_watch(
            p[33].fd, cfgPathname,
            IN_MODIFY | IN_IGNORED);
          p[33].events = POLLIN;
          pinConfigUnload();
          pinConfigLoad();
          setup();
        } else {
          // Some other file -- disregard
        }
      }

      bufPos += sizeof(struct inotify_event) + ev->len;
    }
  }
}


// Init and main loop ------------------------------------------------------

int main(int argc, char *argv[]) {

  char               c;            // Pin input value ('0'/'1')
  int                fd,           // For mmap, sysfs
                     i,            // Generic counter
                     timeout = -1, // poll() timeout
                     lastKey = -1; // Last key down (for repeat)
  uint32_t           pressMask[MAXHWDWORD]; // For Vulcan pinch detect
  struct input_event keyEv, synEv; // uinput events
  sigset_t           sigset;       // Signal mask

  // If in foreground, set max debug level (config may override)
  if(getpgrp() == tcgetpgrp(STDOUT_FILENO)) startupDebug = debug = 99;

  // Locate configuration file (if any) and path ---------------------

  if(argc > 1) { // First argument (if given) is config file name
    char *ptr = strrchr(argv[1], '/'); // Full pathname given?
    if(ptr) { // Pathname given; separate into path & name
      if(!(cfgPathname = strdup(argv[1])))
        err("malloc() fail");
      int len = ptr - argv[1]; // Length of path component
      if(!len) { // Root path?
        cfgPath = "/";
        cfgName = &cfgPathname[1];
      } else {
        if(!(cfgPath = (char *)malloc(len + 1)))
          err("malloc() fail");
        memcpy(cfgPath, argv[1], len);
        cfgPath[len] = 0;
      }
    } else { // No path given; use /boot directory.
      cfgPath = "/boot";
      if(!(cfgPathname = (char *)malloc(
        strlen(cfgPath) + strlen(argv[1]) + 2)))
        err("malloc() fail");
      sprintf(cfgPathname, "%s/%s", cfgPath, argv[1]);
    }
  } else {
    // No argument passed -- config file is located in /boot,
    // name is "[program name].cfg" (e.g. /boot/retrogame.cfg)
    cfgPath = "/boot";
    if(!(cfgPathname = (char *)malloc(
      strlen(cfgPath) + strlen(__progname) + 6)))
      err("malloc() fail");
    sprintf(cfgPathname, "%s/%s.cfg", cfgPath, __progname);
  }
  if(!cfgName) cfgName = &cfgPathname[strlen(cfgPath) + 1];

  if(debug) {
    printf("%s: Config file is '%s'\n", __progname, cfgPathname);
  }

  // Catch signals, config file changes ------------------------------

  // Clear all descriptors and GPIO state, init input event structures
  memset(p, 0, sizeof(p));
  for(i=0; i<35; i++)  p[i].fd = -1;
  for(i=0; i<=MAXHWDWORD*32; i++) key[i] = KEY_RESERVED;
  memset(intstate  , 0, sizeof(intstate));
  memset(extstate  , 0, sizeof(extstate));
  memset(vulcanMask, 0, sizeof(vulcanMask));
  memset(mcpI2C    , 0, sizeof(mcpI2C));
  memset(i2cfd     , 0, sizeof(i2cfd));
  memset(&keyEv    , 0, sizeof(keyEv));
  memset(&synEv    , 0, sizeof(synEv));
  keyEv.type = EV_KEY;
  synEv.type = EV_SYN;
  synEv.code = SYN_REPORT;
  mcpMask    = 0;

  sigfillset(&sigset);
  sigprocmask(SIG_BLOCK, &sigset, NULL);
  // pollfd #32 catches signals, so GPIO cleanup on exit is possible
  p[32].fd     = signalfd(-1, &sigset, 0);
  p[32].events = POLLIN;

  // pollfd #33 and #34 will be used for detecting changes in the
  // config file and its parent directory.  This will let you edit
  // the config and have immediate feedback without needing to kill
  // the process or reboot the system.
  for(i=33; i<=34; i++) {
    p[i].fd     = inotify_init();
    p[i].events = POLLIN;
  }
  fileWatch = inotify_add_watch(p[33].fd, cfgPathname,
    IN_MODIFY | IN_IGNORED);
  inotify_add_watch(p[34].fd, cfgPath,
    IN_CREATE | IN_MOVED_FROM | IN_MOVED_TO);

  // p[0-31] are related to GPIO states, and will be reconfigured
  // each time the config file is loaded.

  // GPIO startup ----------------------------------------------------

  isEarlyPi = earlyPiDetect();
  if(debug & isEarlyPi) {
    printf("%s: running on Rev1 Pi 1 Board\n", __progname);
  }

  // Although Sysfs provides solid GPIO interrupt handling, there's
  // no interface to the internal pull-up resistors (this is by
  // design, being a hardware-dependent feature).  It's necessary to
  // grapple with the GPIO configuration registers directly to enable
  // the pull-ups.  Based on GPIO example code by Dom and Gert van
  // Loo on elinux.org
  if((fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0)
    err("Can't open /dev/mem");
  gpio = mmap(            // Memory-mapped I/O
    NULL,                 // Any adddress will do
    BLOCK_SIZE,           // Mapped block length
    PROT_READ|PROT_WRITE, // Enable read+write
    MAP_SHARED,           // Shared with other processes
    fd,                   // File to map
    bcm_host_get_peripheral_address() + GPIO_BASE);
  close(fd);              // Not needed after mmap()
  if(gpio == MAP_FAILED) err("Can't mmap()");

  pinConfigLoad();
  setup();                   // matrix oriented keyboards

  // Main loop -------------------------------------------------------

  // Monitor GPIO file descriptors for button events.  The poll()
  // function watches for GPIO IRQs in this case; it is NOT
  // continually polling the pins!  Processor load is near zero.

  if(debug) printf("%s: Entering main loop\n", __progname);

  // As in the pinConfigLoad() function, the nesting here gets
  // pretty deep, please excuse the mid-function shift here to
  // 2-space indenting.

  while(running) { // Signal handler will set this to 0 to exit
    // Wait for IRQ on pin (or timeout for button debounce)
    c = 0; // By default, don't issue SYN event
//    if (timeout <= 0)  timeout = 25; // to allow matrix keyboard
//    if(poll(p, 35, timeout) > 0) { // If IRQ...
    if(poll(p, 35, 25) > 0) { // If IRQ...
      if(debug >= 3) printf("%s: handling interrupt\n", __progname);
      for(i=0; i<32; i++) {  // For each GPIO bit...
        if(p[i].revents) { // Event received?
          if(mcpI2C[i]) { // Is port expander (0x20-0x27)
            uint8_t c, buf[4], idx = mcpI2C[i] - 0x20; // 0-7
            // Must drain fd every time else it triggers forever
            lseek(p[i].fd, 0, SEEK_SET);
            while(read(p[i].fd, &c, 1) > 0); // Ignore value
            write(i2cfd[idx], &readAddr, 1);
            if(read(i2cfd[idx], buf, 4) == 4) { // INTCAP+GPIO
              // Buttons pull GPIO low, so invert into intstate[]
              uint16_t merged = ~((buf[3] << 8) | buf[2]);
              uint8_t  i2     = 1 + idx / 2; // Index of 32-bit state
              if(idx & 1) { // Upper half of state
                intstate[i2] = (intstate[i2]&0x0000FFFF)|(merged<<16);
              } else {      // Lower half of state
                intstate[i2] = (intstate[i2]&0xFFFF0000)|merged;
              }
            }
          } else { // Is regular GPIO
            if(debug >= 3) printf("%s: start GPIO for pin %d\n", __progname, i);
            // Read current pin state, store in internal state flag,
            // flag, but don't issue to uinput yet -- must debounce!
            lseek(p[i].fd, 0, SEEK_SET);
            read(p[i].fd, &c, 1);
            if(c == '0')      intstate[0] |=  (1 << i);
            else if(c == '1') intstate[0] &= ~(1 << i);
          }
          timeout      = debounceTime;
          p[i].revents = 0;
        }
      }
      for(; i<35; i++) { // Check signals, etc.
        if(p[i].revents) { // Event received?
          if(debug >= 3) printf("%s: handling sys interrupt\n", __progname);
          pollHandler(i);
          p[i].revents = 0;
        }
      }
      // Else timeout occurred
    } else if(timeout == debounceTime) { // Debounce timeout
      if(debug >= 3) printf("%s: start timeout debounce\n", __progname);
      memset(pressMask, 0, sizeof(pressMask));
      uint8_t  a;
      uint32_t b;
      for(a=i=0; a<MAXHWDWORD; a++) {
        for(b=1; b; b <<= 1, i++) { // i=0 to 159, now even to 191
          if((key[i] > KEY_RESERVED) && (key[i] < GND)) {
            // Compare internal state against previously-issued value.
            // Send keys only for changed states.
            if((intstate[a] & b) != (extstate[a] & b)) {
              // Man this is starting to get ugly.  Y'know this could
              // be done by typedefing a structure with bit fields...
              // it'd be doing about the same thing behind the scenes,
              // but might be more legible in source form.
              extstate[a] = (extstate[a] & ~b) | (intstate[a] & b);
              keyEv.code  = key[i];
              keyEv.value = ((intstate[a] & b) > 0);
              write(keyfd, &keyEv, sizeof(keyEv));
              c = 1; // Follow w/SYN event
              if(intstate[a] & b) { // Press?
                // Note pressed key and set initial repeat interval.
                lastKey = i;
                timeout = repTime1;
                if(debug >= 3) {
                  printf("%s: GPIO%02d key press code %d\n",
                    __progname, i, key[i]);
                }
              } else { // Release?
                // Stop repeat and return to normal IRQ monitoring
                // (no timeout).
                lastKey = timeout = -1;
                if(debug >= 3) {
                  printf("%s: GPIO%02d key release code %d\n",
                    __progname, i, key[i]);
                }
              }
            }
            if(intstate[a] & b) pressMask[a] |= b;
          } else if(debug >= 1 && (intstate[a] & b)) {
            printf("%s: ERROR debounce key=%d, a=%d, b=%d\n", __progname,
                   key[i], a, b);
          }
        }
      }
      if(debug >= 3) printf("%s: end of key puncher\n", __progname);

      // There's an occasional case where it seems the MCP will
      // trigger a pin-change IRQ but then the GPIO pin state
      // reverts to its prior value due to switch bounce; this
      // fails to activate the press *or* release cases above,
      // and the debounce timeout is never reset.  Test 'c'
      // (SYN event flag) as timeout reset fallback.  If not
      // reset, the debounce code above is called on every pass
      // regardless whether input is received, wasting CPU cycles.
      if(!c) timeout = -1;

      // If the "Vulcan nerve pinch" buttons are pressed,
      // set long timeout -- if this time elapses without
      // a button state change, esc keypress will be sent.
      if(key[160] != KEY_RESERVED) { // Any vulcan key defined?
        for(a=0; (a<MAXHWDWORD) &&
         ((pressMask[a] & vulcanMask[a]) == vulcanMask[a]); a++);
        if(a == MAXHWDWORD) timeout = vulcanTime;
      }
    } else if(timeout == vulcanTime) { // Vulcan key timeout
      if(debug >= 3) printf("%s: start vulcan\n", __progname);
      // Send keycode (MAME exits or displays exit menu)
      keyEv.code = key[160];
      if(debug >= 3) {
        printf("%s: GPIO combo %04X%04X%04X%04X%04X press, "
          "release code %d\n", __progname, vulcanMask[4],
          vulcanMask[3], vulcanMask[2], vulcanMask[1], vulcanMask[0],
          key[160]);
      }
      if(debug >= 3) printf("%s: start key impulse\n", __progname);
      for(i=1; i>= 0; i--) { // Press, release
        keyEv.value = i;
        write(keyfd, &keyEv, sizeof(keyEv));
        usleep(10000); // Be slow, else MAME flakes
        write(keyfd, &synEv, sizeof(synEv));
        usleep(10000);
      }
      timeout = -1; // Return to normal processing
      c       = 0;  // No add'l SYN required
    } else if(lastKey >= 0) { // Else key repeat timeout
      if(debug >= 3) printf("%s: start last key repeat\n", __progname);
      if(timeout == repTime1) timeout = repTime2;
      else if(timeout > 30)   timeout -= 5; // Accelerate
      c           = 1; // Follow w/SYN event
      keyEv.code  = key[lastKey];
      keyEv.value = 2; // Key repeat event
      if(debug >= 3) {
        printf("%s: repeating key code %d\n",
          __progname, keyEv.code);
      }
      write(keyfd, &keyEv, sizeof(keyEv));
    }

    if (loop())  timeout = debounceTime;

    if(c) write(keyfd, &synEv, sizeof(synEv));
  }  // end of while

  // Clean up --------------------------------------------------------

  pinConfigUnload(); // Close uinput, un-export pins

  if(debug) printf("%s: Done.\n", __progname);

  return 0;
}
