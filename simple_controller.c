#include "includes/i2c.h"
#include "includes/led.h"
#include <msp430fr5994.h>


int main(void) {
   uint8_t data[I2C_MAX_BUFFER_SIZE] = {0};
   unsigned int address;
   int numBytes;

   WDTCTL = WDTPW | WDTHOLD;

   // Disable the GPIO power-on default high-impedance mode to activate
   // previously configured port settings
   PM5CTL0 &= ~LOCKLPM5;

   setup_LEDs();
   init_controller();
   __bis_SR_register(GIE); // Enable interrupts

   while (1) {
         // address 1 should blink 1 then 2 times
         address = 1;
         numBytes = 3;
         if (i2c_read(address, data, numBytes)) {
            display_led_count(data, numBytes);
         }
         // address 2 should not return bytes. Not available on target
         address = 2;
         numBytes = 2;
         if (i2c_read(address, data, numBytes)) {
            display_led_count(data, numBytes);
         }
         /*
         // address 3 should blink 3 times.
         address = 3;
         numBytes = 1;
         if (i2c_read(address, data, numBytes)) {
            display_led_count(data, numBytes);
         }
         */
         // address 4 should blink 4 times
         address = 4;
         numBytes = 1;
         if (i2c_read(address, data, numBytes)) {
            display_led_count(data, numBytes);
         }

   }
}
