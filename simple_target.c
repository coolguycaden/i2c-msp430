#include "includes/i2c.h"
#include "includes/led.h"
#include <msp430fr5994.h>
#include <stddef.h>
#include <stdint.h>

#define NUM_MOCKS 3
#define DATA1LEN 3
#define DATA2LEN 1
#define DATA3LEN 1

int main(void) {
   WDTCTL = WDTPW | WDTHOLD;

   // Disable the GPIO power-on default high-impedance mode to activate
   // previously configured port settings
   PM5CTL0 &= ~LOCKLPM5;

   setup_LEDs();

   uint8_t data1[DATA1LEN] = {1, 2, 3};
   uint8_t data2[DATA2LEN] = {4};
   uint8_t data3[DATA3LEN] = {3};

   MockAddress mocks[NUM_MOCKS] = {
       {.address = 0x1, .regx = I2C_NO_REG, .data = data1, .dataLen = DATA1LEN},
       {.address = 0x4, .regx = I2C_NO_REG, .data = data2, .dataLen = DATA2LEN},
       {.address = 0x3, .regx = 0x2, .data = data3, .dataLen = DATA3LEN},
   };

   init_target(mocks, NUM_MOCKS);

   // sleep and use interrupt handlers
   __bis_SR_register(GIE | LPM0_bits);
}