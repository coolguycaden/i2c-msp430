#ifndef I2C_H
#define I2C_H

#include <msp430fr5994.h>
#include <stdbool.h>
#include <stdint.h>

#define I2C_MAX_BUFFER_SIZE 256

#define I2C_NO_REG 0xFF

typedef struct {
   uint8_t address;
   uint8_t regx;
   uint8_t * data; //array
   int dataLen;
} MockAddress;

void init_controller(void);
void init_target(MockAddress * mocks, int mockLen);
bool i2c_read(unsigned int targ_addr, uint8_t data[], unsigned int dataLen);
void __attribute__((interrupt(EUSCI_B2_VECTOR))) USCI_B2_ISR(void);

#endif // I2C_H