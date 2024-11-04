#ifndef LED_H
#define LED_H

#include <stdint.h>
#include <stdbool.h>
#include <msp430fr5994.h>

#define RED_LED BIT0
#define GRN_LED BIT1
#define TIME_DELAY 150000

void setup_LEDs(void);
void toggle_led(unsigned int color);
void set_led(bool on, unsigned int color);
void display_led_count(uint8_t sequence[], int seqLen);

#endif // LED_H