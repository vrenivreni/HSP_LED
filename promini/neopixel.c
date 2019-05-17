/*-------------------------------------------------------------------------
  Arduino library to control a wide variety of WS2811- and WS2812-based RGB
  LED devices such as Adafruit FLORA RGB Smart Pixels and NeoPixel strips.
  Currently handles 400 and 800 KHz bitstreams on 8, 12 and 16 MHz ATmega
  MCUs, with LEDs wired for various color orders.  Handles most output pins
  (possible exception with upper PORT registers on the Arduino Mega).
  Written by Phil Burgess / Paint Your Dragon for Adafruit Industries,
  contributions by PJRC, Michael Miller and other members of the open
  source community.
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!
  -------------------------------------------------------------------------
  This file is part of the Adafruit NeoPixel library.
  NeoPixel is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation, either version 3 of
  the License, or (at your option) any later version.
  NeoPixel is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with NeoPixel.  If not, see
  <http://www.gnu.org/licenses/>.
  -------------------------------------------------------------------------*/

#include <string.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include "neopixel.h"
#include "wait.h"


  int8_t
  #ifdef NEO_KHZ400  // If 400 KHz NeoPixel support enabled...
  is800KHz=1,      // ...true if 800 KHz pixels
  #endif
  begun=0;         // true if begin() previously called
  uint16_t
  numLEDs,       // Number of RGB LEDs in strip
  numBytes=3*150;      // Size of 'pixels' buffer below (3 or 4 bytes/pixel)
  uint8_t pixMem[150*3] = {0};
  int8_t
  pin;           // Output pin number (-1 if not yet set)
  uint8_t
  brightness,
  *pixels=pixMem,        // Holds LED color values (3 or 4 bytes each)
  rOffset,       // Index of red byte within each 3- or 4-byte pixel
  gOffset,       // Index of green byte
  bOffset,       // Index of blue byte
  wOffset;       // Index of white byte (same as rOffset if no white)
  uint32_t
  endTime;       // Latch timing reference
  volatile uint8_t
  *port;         // Output PORT register
  uint8_t
  pinMask;       // Output PORT bitmask

void NeoPixelBegin(int16_t numLED) {
	if (begun) return;
    port    = &PORTD;
    pinMask = 1 << PD6;
	PORTD &= ~pinMask;
	begun = 1;
	brightness = 0;
	endTime = 0;
	numLEDs = numLED;
	wOffset = (NEO_GRB >> 6) & 0b11; // See notes in header file
	rOffset = (NEO_GRB >> 4) & 0b11; // regarding R/G/B/W offsets
	gOffset = (NEO_GRB >> 2) & 0b11;
	bOffset =  NEO_GRB       & 0b11;
	waitForMs(20);
}





void NeoPixelShow(void) {

  // Data latch = 300+ microsecond pause in the output stream.  Rather than
  // put a delay at the end of the function, the ending time is noted and
  // the function will simply hold off (if needed) on issuing the
  // subsequent round of data until the latch time has elapsed.  This
  // allows the mainline code to start generating the next frame of data
  // rather than stalling for the latch.
  waitForUs(300);
  // endTime is a private member (rather than global var) so that multiple
  // instances on different pins can be quickly issued in succession (each
  // instance doesn't delay the next).

  // In order to make this code runtime-configurable to work with any pin,
  // SBI/CBI instructions are eschewed in favor of full PORT writes via the
  // OUT or ST instructions.  It relies on two facts: that peripheral
  // functions (such as PWM) take precedence on output pins, so our PORT-
  // wide writes won't interfere, and that interrupts are globally disabled
  // while data is being issued to the LEDs, so no other code will be
  // accessing the PORT.  The code takes an initial 'snapshot' of the PORT
  // state, computes 'pin high' and 'pin low' values, and writes these back
  // to the PORT register as needed.

  // NRF52 may use PWM + DMA (if available), may not need to disable interrupt

  cli(); // Need 100% focus on instruction timing

#ifdef __AVR__
// AVR MCUs -- ATmega & ATtiny (no XMEGA) ---------------------------------

  volatile uint16_t
    i   = numBytes; // Loop counter
  volatile uint8_t
   *ptr = pixels,   // Pointer to next byte
    b   = *ptr++,   // Current byte value
    hi,             // PORT w/output bit set high
    lo;             // PORT w/output bit set low

  // Hand-tuned assembly code issues data to the LED drivers at a specific
  // rate.  There's separate code for different CPU speeds (8, 12, 16 MHz)
  // for both the WS2811 (400 KHz) and WS2812 (800 KHz) drivers.  The
  // datastream timing for the LED drivers allows a little wiggle room each
  // way (listed in the datasheets), so the conditions for compiling each
  // case are set up for a range of frequencies rather than just the exact
  // 8, 12 or 16 MHz values, permitting use with some close-but-not-spot-on
  // devices (e.g. 16.5 MHz DigiSpark).  The ranges were arrived at based
  // on the datasheet figures and have not been extensively tested outside
  // the canonical 8/12/16 MHz speeds; there's no guarantee these will work
  // close to the extremes (or possibly they could be pushed further).
  // Keep in mind only one CPU speed case actually gets compiled; the
  // resulting program isn't as massive as it might look from source here.


// 16 MHz(ish) AVR --------------------------------------------------------

#ifdef NEO_KHZ400 // 800 KHz check needed only if 400 KHz support enabled
  if(is800KHz) {
#endif

    // WS2811 and WS2812 have different hi/lo duty cycles; this is
    // similar but NOT an exact copy of the prior 400-on-8 code.

    // 20 inst. clocks per bit: HHHHHxxxxxxxxLLLLLLL
    // ST instructions:         ^   ^        ^       (T=0,5,13)

    volatile uint8_t next, bit;

    hi   = *port |  pinMask;
    lo   = *port & ~pinMask;
    next = lo;
    bit  = 8;

    asm volatile(
     "head20:"                   "\n\t" // Clk  Pseudocode    (T =  0)
      "st   %a[port],  %[hi]"    "\n\t" // 2    PORT = hi     (T =  2)
      "sbrc %[byte],  7"         "\n\t" // 1-2  if(b & 128)
       "mov  %[next], %[hi]"     "\n\t" // 0-1   next = hi    (T =  4)
      "dec  %[bit]"              "\n\t" // 1    bit--         (T =  5)
      "st   %a[port],  %[next]"  "\n\t" // 2    PORT = next   (T =  7)
      "mov  %[next] ,  %[lo]"    "\n\t" // 1    next = lo     (T =  8)
      "breq nextbyte20"          "\n\t" // 1-2  if(bit == 0) (from dec above)
      "rol  %[byte]"             "\n\t" // 1    b <<= 1       (T = 10)
      "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 12)
      "nop"                      "\n\t" // 1    nop           (T = 13)
      "st   %a[port],  %[lo]"    "\n\t" // 2    PORT = lo     (T = 15)
      "nop"                      "\n\t" // 1    nop           (T = 16)
      "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 18)
      "rjmp head20"              "\n\t" // 2    -> head20 (next bit out)
     "nextbyte20:"               "\n\t" //                    (T = 10)
      "ldi  %[bit]  ,  8"        "\n\t" // 1    bit = 8       (T = 11)
      "ld   %[byte] ,  %a[ptr]+" "\n\t" // 2    b = *ptr++    (T = 13)
      "st   %a[port], %[lo]"     "\n\t" // 2    PORT = lo     (T = 15)
      "nop"                      "\n\t" // 1    nop           (T = 16)
      "sbiw %[count], 1"         "\n\t" // 2    i--           (T = 18)
       "brne head20"             "\n"   // 2    if(i != 0) -> (next byte)
      : [port]  "+e" (port),
        [byte]  "+r" (b),
        [bit]   "+r" (bit),
        [next]  "+r" (next),
        [count] "+w" (i)
      : [ptr]    "e" (ptr),
        [hi]     "r" (hi),
        [lo]     "r" (lo));

#ifdef NEO_KHZ400
  } else { // 400 KHz

    // The 400 KHz clock on 16 MHz MCU is the most 'relaxed' version.

    // 40 inst. clocks per bit: HHHHHHHHxxxxxxxxxxxxLLLLLLLLLLLLLLLLLLLL
    // ST instructions:         ^       ^           ^         (T=0,8,20)

    volatile uint8_t next, bit;

    hi   = *port |  pinMask;
    lo   = *port & ~pinMask;
    next = lo;
    bit  = 8;

    asm volatile(
     "head40:"                  "\n\t" // Clk  Pseudocode    (T =  0)
      "st   %a[port], %[hi]"    "\n\t" // 2    PORT = hi     (T =  2)
      "sbrc %[byte] , 7"        "\n\t" // 1-2  if(b & 128)
       "mov  %[next] , %[hi]"   "\n\t" // 0-1   next = hi    (T =  4)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T =  6)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T =  8)
      "st   %a[port], %[next]"  "\n\t" // 2    PORT = next   (T = 10)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 12)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 14)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 16)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 18)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 20)
      "st   %a[port], %[lo]"    "\n\t" // 2    PORT = lo     (T = 22)
      "nop"                     "\n\t" // 1    nop           (T = 23)
      "mov  %[next] , %[lo]"    "\n\t" // 1    next = lo     (T = 24)
      "dec  %[bit]"             "\n\t" // 1    bit--         (T = 25)
      "breq nextbyte40"         "\n\t" // 1-2  if(bit == 0)
      "rol  %[byte]"            "\n\t" // 1    b <<= 1       (T = 27)
      "nop"                     "\n\t" // 1    nop           (T = 28)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 30)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 32)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 34)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 36)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 38)
      "rjmp head40"             "\n\t" // 2    -> head40 (next bit out)
     "nextbyte40:"              "\n\t" //                    (T = 27)
      "ldi  %[bit]  , 8"        "\n\t" // 1    bit = 8       (T = 28)
      "ld   %[byte] , %a[ptr]+" "\n\t" // 2    b = *ptr++    (T = 30)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 32)
      "st   %a[port], %[lo]"    "\n\t" // 2    PORT = lo     (T = 34)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 36)
      "sbiw %[count], 1"        "\n\t" // 2    i--           (T = 38)
      "brne head40"             "\n"   // 1-2  if(i != 0) -> (next byte)
      : [port]  "+e" (port),
        [byte]  "+r" (b),
        [bit]   "+r" (bit),
        [next]  "+r" (next),
        [count] "+w" (i)
      : [ptr]    "e" (ptr),
        [hi]     "r" (hi),
        [lo]     "r" (lo));
  }
#endif // NEO_KHZ400

// END AVR ----------------------------------------------------------------


#else 
#error Architecture not supported
#endif


// END ARCHITECTURE SELECT ------------------------------------------------

  sei();


  //endTime = micros(); // Save EOD time for latch on next call
}

// Set the output pin number
void setPin() {
    port    = &PORTD;
    pinMask = 1 << PD6;
}

// Set pixel color from separate R,G,B components:
void setPixelColor(
 uint16_t n, uint8_t r, uint8_t g, uint8_t b) {

  if(n < numLEDs) {
    if(brightness) { // See notes in setBrightness()
      r = (r * brightness) >> 8;
      g = (g * brightness) >> 8;
      b = (b * brightness) >> 8;
    }
    uint8_t *p;
    if(wOffset == rOffset) { // Is an RGB-type strip
      p = &pixels[n * 3];    // 3 bytes per pixel
    } else {                 // Is a WRGB-type strip
      p = &pixels[n * 4];    // 4 bytes per pixel
      p[wOffset] = 0;        // But only R,G,B passed -- set W to 0
    }
    p[rOffset] = r;          // R,G,B always stored
    p[gOffset] = g;
    p[bOffset] = b;
  }
}

// Query color from previously-set pixel (returns packed 32-bit RGB value)
uint32_t getPixelColor(uint16_t n) {
  if(n >= numLEDs) return 0; // Out of bounds, return no color.

  uint8_t *p;

  if(wOffset == rOffset) { // Is RGB-type device
    p = &pixels[n * 3];
    if(brightness) {
      // Stored color was decimated by setBrightness().  Returned value
      // attempts to scale back to an approximation of the original 24-bit
      // value used when setting the pixel color, but there will always be
      // some error -- those bits are simply gone.  Issue is most
      // pronounced at low brightness levels.
      return (((uint32_t)(p[rOffset] << 8) / brightness) << 16) |
             (((uint32_t)(p[gOffset] << 8) / brightness) <<  8) |
             ( (uint32_t)(p[bOffset] << 8) / brightness       );
    } else {
      // No brightness adjustment has been made -- return 'raw' color
      return ((uint32_t)p[rOffset] << 16) |
             ((uint32_t)p[gOffset] <<  8) |
              (uint32_t)p[bOffset];
    }
  } else {                 // Is RGBW-type device
    p = &pixels[n * 4];
    if(brightness) { // Return scaled color
      return (((uint32_t)(p[wOffset] << 8) / brightness) << 24) |
             (((uint32_t)(p[rOffset] << 8) / brightness) << 16) |
             (((uint32_t)(p[gOffset] << 8) / brightness) <<  8) |
             ( (uint32_t)(p[bOffset] << 8) / brightness       );
    } else { // Return raw color
      return ((uint32_t)p[wOffset] << 24) |
             ((uint32_t)p[rOffset] << 16) |
             ((uint32_t)p[gOffset] <<  8) |
              (uint32_t)p[bOffset];
    }
  }
}

// Returns pointer to pixels[] array.  Pixel data is stored in device-
// native format and is not translated here.  Application will need to be
// aware of specific pixel data format and handle colors appropriately.
uint8_t *getPixels(void) {
  return pixels;
}


/*uint8_t getBrightness() {
	return brightness;
}*/
// Adjust output brightness; 0=darkest (off), 255=brightest.  This does
// NOT immediately affect what's currently displayed on the LEDs.  The
// next call to show() will refresh the LEDs at this level.  However,
// this process is potentially "lossy," especially when increasing
// brightness.  The tight timing in the WS2811/WS2812 code means there
// aren't enough free cycles to perform this scaling on the fly as data
// is issued.  So we make a pass through the existing color data in RAM
// and scale it (subsequent graphics commands also work at this
// brightness level).  If there's a significant step up in brightness,
// the limited number of steps (quantization) in the old data will be
// quite visible in the re-scaled version.  For a non-destructive
// change, you'll need to re-render the full strip data.  C'est la vie.
void setBrightness(uint8_t b) {
  // Stored brightness value is different than what's passed.
  // This simplifies the actual scaling math later, allowing a fast
  // 8x8-bit multiply and taking the MSB.  'brightness' is a uint8_t,
  // adding 1 here may (intentionally) roll over...so 0 = max brightness
  // (color values are interpreted literally; no scaling), 1 = min
  // brightness (off), 255 = just below max brightness.
  uint8_t newBrightness = b + 1;
  if(newBrightness != brightness) { // Compare against prior value
    // Brightness has changed -- re-scale existing data in RAM
    uint8_t  c,
            *ptr           = pixels,
             oldBrightness = brightness - 1; // De-wrap old brightness value
    uint16_t scale;
    if(oldBrightness == 0) scale = 0; // Avoid /0
    else if(b == 255) scale = 65535 / oldBrightness;
    else scale = (((uint16_t)newBrightness << 8) - 1) / oldBrightness;
    for(uint16_t i=0; i<numBytes; i++) {
      c      = *ptr;
      *ptr++ = (c * scale) >> 8;
    }
    brightness = newBrightness;
  }
}

//Return the brightness value
uint8_t getBrightness(void) {
  return brightness - 1;
}

void NeoPixelClear() {
  memset(pixels, 0, numBytes);
}

