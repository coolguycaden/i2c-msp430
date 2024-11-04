#include <msp430fr5994.h>

#include "led.h"

void setup_LEDs(void) {
   P1DIR |= RED_LED;  // set pin 1.0 as output
   P1OUT &= ~RED_LED; // clear pin 1.0 (make it low)

   P1DIR |= GRN_LED;  // set pin 1.1 as output
   P1OUT &= ~GRN_LED; // clearn pin 1.1 (make it low)
}

void toggle_led(unsigned int color) {
   P1OUT ^= color;
}

void set_led(bool on, unsigned int color) {
   if (on) {
      P1OUT |= color;
   } else {
      P1OUT &= ~color;
   }
}

void display_led_count(uint8_t sequence[], int seqLen) {
   unsigned int codeColor = GRN_LED;
   unsigned int wordColor = RED_LED;
   int blinks;
   set_led(true, wordColor);
   __delay_cycles(TIME_DELAY);

   // do blinks for the word
   for (int i = 0; i < seqLen; i++) {
      blinks = sequence[i];
      for (int b = 0; b < blinks; b++) {
         set_led(false, codeColor);
         __delay_cycles(TIME_DELAY);
         set_led(true, codeColor);
         __delay_cycles(TIME_DELAY);
      }
      // toggle the separator
      set_led(false, codeColor);
      __delay_cycles(TIME_DELAY*3);
   }

   // done blinking the word
   set_led(false, wordColor);
   __delay_cycles(TIME_DELAY * 8);
}