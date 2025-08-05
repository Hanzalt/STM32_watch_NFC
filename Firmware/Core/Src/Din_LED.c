/*
 * Din_LED.c
 *
 *  Created on: Apr 20, 2024
 *      Author: vakhaib
 */

#include "Din_LED.h"

void clear_LEDs(LEDs* leds) {

	for (int i = 0; i < 12; i++) {

		for (int j = 0; j < 8; j++) {
			(leds + i) ->r[j] = T0H;
			(leds + i) ->g[j] = T0H;
			(leds + i) ->b[j] = T0H;
			if (i >= 0 && i <= 2) {
				(leds + 12 + i) ->r[j] = 0;
				(leds + 12 + i) ->g[j] = 0;
				(leds + 12 + i) ->b[j] = 0;
			}
		}
	}
}
	/*
	leds -> pause[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,0,0};
	*/
	//memset(leds -> pause, 0x00, 35);



void turn_spec_LEDs(LEDs* leds, rgb_color* colors) {
	for (int i = 0; i < 12; i++) {
		uint8_t brightness = colors[i].a; // 0–255
		uint8_t r = (colors[i].r * brightness) / 255;
		uint8_t g = (colors[i].g * brightness) / 255;
		uint8_t b = (colors[i].b * brightness) / 255;

		for (int j = 0; j < 8; j++) {
			uint8_t mask = 1 << (7 - j); // MSB first

			(leds + i)->g[j] = (g & mask) ? T1H : T0H;
			(leds + i)->r[j] = (r & mask) ? T1H : T0H;
			(leds + i)->b[j] = (b & mask) ? T1H : T0H;

			if (i >= 0 && i <= 2) {
				(leds + 12 + i)->r[j] = 0;
				(leds + 12 + i)->g[j] = 0;
				(leds + 12 + i)->b[j] = 0;
			}
		}
	}
}

void turn_LEDs(LEDs* leds, uint16_t num_LEDs) {

	for (int i = 0; i < num_LEDs; i++) {
		for (int j = 0; j < 8; j++) {
			if (j>=2) {
				(leds + i) ->r[j] = T1H;
				(leds + i) ->g[j] = T1H;
				(leds + i) ->b[j] = T1H;
				if (i==0) {
					(leds + i) ->b[j] = T0H;
				}
				if (i >= 0 && i <= 2) {
					(leds + num_LEDs + i) ->r[j] = 0;
					(leds + num_LEDs + i) ->g[j] = 0;
					(leds + num_LEDs + i) ->b[j] = 0;
				}
			} else {
				(leds + i) ->r[j] = T0H;
				(leds + i) ->g[j] = T0H;
				(leds + i) ->b[j] = T0H;
				if (i >= 0 && i <= 2) {
					(leds + num_LEDs + i) ->r[j] = 0;
					(leds + num_LEDs + i) ->g[j] = 0;
					(leds + num_LEDs + i) ->b[j] = 0;
				}
			}
		}
	}
}

static int decToBinary(int n) {
	// array to store binary number
	int result = 0;
	int binaryNum[1000];

	// counter for binary array
	int i = 0;
	while (n > 0) {

		// storing remainder in binary array
		binaryNum[i] = n % 2;
		n = n / 2;
		i++;
	}

	// printing binary array in reverse order
	for (int j = i - 1; j >= 0; j--)
		result = result*10 + binaryNum[j];
	return result;
}


