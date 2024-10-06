/*
 * Din_LED.h
 *
 *  Created on: Apr 20, 2024
 *      Author: vakhaib
 */

#ifndef INC_DIN_LED_H_
#define INC_DIN_LED_H_
#include <stdio.h>
#include <stdint.h>
#include "main.h"

typedef struct {
	uint8_t r;
	uint8_t g;
	uint8_t b;
	uint8_t a;
} rgb_color;

typedef struct {
	uint16_t g[8];
	uint16_t r[8];
	uint16_t b[8];
	//uint16_t pause[65];
} LEDs;

#define T1H  20
#define T0H  5

//USABLE
void clear_LEDs(LEDs* leds, uint16_t num_LEDs); // Looks ugly on oscilloscope - do not use
void set_LED(LEDs* leds, uint16_t pos_LED, rgb_color color);
void turn_spec_LEDs(LEDs* leds, rgb_color* colors);

//DEBUG
void turn_LEDs(LEDs* leds, uint16_t num_LEDs);

//PRIVATE
static void led_manager();
static int decToBinary(int n);
#endif /* INC_DIN_LED_H_ */
