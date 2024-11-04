#include "i2c.h"
#include "led.h"
#include <stddef.h>

// types and enums hidden. Only needed by this file
typedef struct {
   unsigned int baud_scale;
   unsigned int count;
   unsigned int targ_addr;
   int mode;
   int role;
} i2c_config;

typedef enum {
   I2C_RX,
   I2C_TX
} I2C_Modes;

typedef enum {
   I2C_CTLR,
   I2C_TARG
} I2C_Roles;

typedef enum {
   I2C_IDLE,
   I2C_PENDING,
   I2C_READY,
} I2C_Processes;

// state machine variables
unsigned int i2c_process = I2C_IDLE;

// globals with scope only in this file for setting and getting I2C data
static uint8_t txBuffer[4][I2C_MAX_BUFFER_SIZE] = {0};
static int txByteLimit[4] = {0};
// static uint8_t tx_buffer0[I2C_MAX_BUFFER_SIZE] = {0};
// static int txByteLimit0 = 0;
// static uint8_t tx_buffer1[I2C_MAX_BUFFER_SIZE] = {0};
// static int txByteLimit1 = 0;
// static uint8_t tx_buffer2[I2C_MAX_BUFFER_SIZE] = {0};
// static int txByteLimit2 = 0;
// static uint8_t tx_buffer3[I2C_MAX_BUFFER_SIZE] = {0};
// static int txByteLimit3 = 0;
static uint8_t rxBuffer[I2C_MAX_BUFFER_SIZE] = {0};
static int rxByteLimit = 0;

/*
   **********************
   Store Mocks for Target
   **********************
*/
static MockAddress * mocks; // array
static int mockLen;         // number of mocks

/*
   *****************
   Utility functions
   *****************
*/

void copyArray(uint8_t * fromArr, uint8_t * toArr, int count) {
   for (int i = 0; i < count; i++) {
      toArr[i] = fromArr[i];
   }
}
/*
   *************************************
   Initializing GPIO and I2C Peripherals
   *************************************
*/

void init_gpio(void) {
   // Configure GPIO
   P7SEL0 |= BIT0 | BIT1;
   P7SEL1 &= ~(BIT0 | BIT1);

   // Pullups only for this example, use external 10k otherwise
   P7REN |= BIT0 | BIT1; // pull up/down enabled
   P7OUT |= BIT0 | BIT1; // pull up enabled
}

/*
   ***************************************
   Configure Registers for Modes and Roles
   ***************************************
*/

void set_config(i2c_config * conf) {
   // guide recommends setting UCSWRST to avoid undefined behaviour
   // while configurin registers
   UCB2CTLW0 |= UCSWRST; // Software reset enabled

   if (conf->role == I2C_CTLR) {
      UCB2BRW = conf->baud_scale; // baudrate = SMCLK / UCB2BRW
      // num bytes expected to be sent
      UCB2TBCNT = conf->count;     // number of bytes to tx/rx
      UCB2I2CSA = conf->targ_addr; // target address
   }
   // set mocks for target
   else {
      for (int m = 0; m < mockLen; m++) {
         uint8_t * mockData = mocks[m].data;
         int mockDataLen = mocks[m].dataLen;
         // own address + enable address
         if (m == 0) {
            UCB2I2COA0 = mocks[0].address;
            UCB2I2COA0 |= UCOAEN;
            copyArray(mockData, txBuffer[0], mockLen);
            txByteLimit[0] = mockDataLen;
         } else if (m == 1) {
            UCB2I2COA1 = mocks[1].address;
            UCB2I2COA1 |= UCOAEN;
            copyArray(mockData, txBuffer[1], mockLen);
            txByteLimit[1] = mockDataLen;
         } else if (m == 2) {
            UCB2I2COA2 = mocks[2].address;
            UCB2I2COA2 |= UCOAEN;
            copyArray(mockData, txBuffer[2], mockLen);
            txByteLimit[2] = mockDataLen;
         } else if (m == 3) {
            UCB2I2COA3 = mocks[3].address;
            UCB2I2COA3 |= UCOAEN;
            copyArray(mockData, txBuffer[3], mockLen);
            txByteLimit[3] = mockDataLen;
         }
      }
   }

   UCB2CTLW0 &= ~UCSWRST; // clear reset register prior to setting interrupts

   // set interrupts by role and mode
   if (conf->role == I2C_CTLR) {
      UCB2IE |= UCNACKIE | UCBCNTIE;
      if (conf->mode == I2C_RX) {
         UCB2IE |= UCRXIE;
      } else {
         UCB2IE |= UCTXIE;
      }
   } else {
      UCB2IE |= UCSTPIE;
      if (conf->mode == I2C_RX) {
         UCB2IE |= UCRXIE;
      } else {
         UCB2IE |= UCTXIE0 | UCTXIE1 | UCTXIE2 | UCTXIE3;
      }
   }
}

void init_controller() {
   init_gpio();
   UCB2CTLW0 |= UCSWRST;                   // Software reset enabled
   UCB2CTLW0 |= UCMODE_3 | UCMST | UCSYNC; // I2C mode, Controller mode, sync
   UCB2CTLW1 |= UCASTP_2;                  // Automatic stop generated
                                           // after UCB2TBCNT is reached
   UCB2CTL1 &= ~UCSWRST;
}

void init_target(MockAddress * mArr, int mLen) {
   init_gpio();

   // save mocks
   mocks = mArr;
   mockLen = mLen;

   UCB2CTLW0 |= UCSWRST;           // Software reset enabled
   UCB2CTLW0 |= UCMODE_3 | UCSYNC; // I2C mode, sync mode

   UCB2CTLW0 &= ~UCSWRST; // clear reset register

   i2c_config conf = {
       .mode = I2C_TX,
       .role = I2C_TARG,
   };

   set_config(&conf);
}

/*
   ************************************
   Setting, clearing, reading registers
   ************************************
*/

void start_condition(void) {
   UCB2CTL1 |= UCTXSTT; // transmit the I2C start condition
}

void stop_condition(void) {
   UCB2CTL1 |= UCTXSTP; // transmit the I2C stop condition
}

void clear_stop_cond_IF(void) {
   UCB2IFG &= ~UCSTPIFG; // Clear stop condition int flag
}

bool waiting_for_stop() {
   return UCB2CTL1 & UCTXSTP;
}

void set_tx_buffer(uint8_t val) {
   UCB2TXBUF = val;
}

uint8_t get_rx_buffer() {
   return UCB2RXBUF;
}

unsigned int get_count() {
   return UCB2TBCNT;
}

/*
   Controller read and write higher level functions
*/

// controller read single byte from target into data variable
bool i2c_read(unsigned int targ_addr, uint8_t data[], unsigned int len) {
   i2c_process = I2C_PENDING;
   rxByteLimit = len;
   i2c_config conf = {
       .baud_scale = 0x10,
       .count = len,
       .targ_addr = targ_addr,
       .mode = I2C_RX,
       .role = I2C_CTLR};
   set_config(&conf);
   start_condition(); // start operation (rx in this case)

   // wait for I2C eUSCI controller to finish processing
   // maybe also good place to put in low power mode with CPU off
   while (i2c_process == I2C_PENDING) {
      __no_operation();
   }

   // Ready means success
   if (i2c_process == I2C_READY) {
      copyArray(rxBuffer, data, len); // fill the data read from target
      i2c_process = I2C_IDLE;
      return true; // true means success
   }
   // Process woutld be IDLE here and something was wrong
   else {
      return false; // false means error
   }
}

/*
   ******************************************
   Interrupt Handler for Controller and Target
   ******************************************
*/

void processTX(int * byteCount, int txNum) {
   // eUSCI module was addressed
   // if no register, target can just transmit bytes
   if (mocks[txNum].regx == I2C_NO_REG) {
      set_tx_buffer(txBuffer[txNum][*byteCount]);
      (*byteCount)++;
      if (*byteCount >= txByteLimit[txNum]) {
         *byteCount = 0;
         i2c_process = I2C_READY;
      }
   }
   // controller should have sent valid register address
   // if so then send the data at the "register"
   else {
      i2c_process = I2C_IDLE; // this is reset condition when invalid
      // need to implement how to handle register
   }
}

// interrupt handles both controller and target
void __attribute__((interrupt(EUSCI_B2_VECTOR))) USCI_B2_ISR(void) {
   static int byteCount = 0;
   switch (__even_in_range(UCB2IV, USCI_I2C_UCBIT9IFG)) {
   case USCI_I2C_UCNACKIFG:
      // target has not acknowledged start conditions from controller
      stop_condition();
      i2c_process = I2C_IDLE;
      break;
   case USCI_I2C_UCSTPIFG:
      // target received stop condition from controller
      clear_stop_cond_IF();

      // target resets state during stop condition
      byteCount = 0;
      i2c_process = I2C_IDLE;
      break;
   case USCI_I2C_UCRXIFG0:
      // byte received, save it to the rx buffer
      rxBuffer[byteCount] = get_rx_buffer();
      byteCount++;
      if (byteCount >= rxByteLimit) {
         byteCount = 0;
         i2c_process = I2C_READY;
      }
      break;
   case USCI_I2C_UCTXIFG0:
      processTX(&byteCount, 0);
      break;
   case USCI_I2C_UCTXIFG1:
      processTX(&byteCount, 1);
      break;
   case USCI_I2C_UCTXIFG2:
      processTX(&byteCount, 2);
      break;
   case USCI_I2C_UCTXIFG3:
      processTX(&byteCount, 3);
      break;
   case USCI_I2C_UCBCNTIFG:
      // interrupts when byte count tx or rx is complete
      // guide admits this is unreliable for decision making
      // avoid using
      break;
   case USCI_I2C_UCBIT9IFG:
      // this is the interrupt triggered when an acknowledge is
      // detected on the bus
      break;
   default:
      break;
   }
}
