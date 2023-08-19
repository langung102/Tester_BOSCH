/*
 * input_reading.c
 *
 *  Created on: Aug 18, 2023
 *      Author: minhl
 */

#include "input_reading.h"

#define NO_OF_BUTTONS					5
#define DURATION_FOR_AUTO_INCREASING	100
#define BUTTON_IS_PRESSED				GPIO_PIN_RESET
#define BUTTON_IS_RELEASED				GPIO_PIN_SET

static	GPIO_PinState buttonBuffer[NO_OF_BUTTONS];
static	GPIO_PinState debounceButtonBuffer1[NO_OF_BUTTONS];
static	GPIO_PinState debounceButtonBuffer2[NO_OF_BUTTONS];
static	GPIO_PinState debounceButtonBuffer3[NO_OF_BUTTONS];
static	uint8_t flagForButtonPress1s[NO_OF_BUTTONS];
static	uint16_t counterForButtonPress1s[NO_OF_BUTTONS];
int button_flag[NO_OF_BUTTONS];

int check_button_flag(int i) {
	if (button_flag[i] == 1) {
		button_flag[i] = 0;
		return 1;
	}
	return 0;
}

void button_reading(void) {
	for (char i=0; i<NO_OF_BUTTONS; i++) {
		debounceButtonBuffer3[i] = debounceButtonBuffer2[i];
		debounceButtonBuffer2[i] = debounceButtonBuffer1[i];
		switch (i) {
			case 0:
				debounceButtonBuffer1[i] = HAL_GPIO_ReadPin(A_GPIO_Port, A_Pin);
				break;
			case 1:
				debounceButtonBuffer1[i] = HAL_GPIO_ReadPin(B_GPIO_Port, B_Pin);
				break;
			case 2:
				debounceButtonBuffer1[i] = HAL_GPIO_ReadPin(C_GPIO_Port, C_Pin);
				break;
			case 3:
				debounceButtonBuffer1[i] = HAL_GPIO_ReadPin(D_GPIO_Port, D_Pin);
				break;
			case 4:
				debounceButtonBuffer1[i] = HAL_GPIO_ReadPin(CTR_GPIO_Port, CTR_Pin);
				break;
			default:
				break;
		}
		if ((debounceButtonBuffer1[i] == debounceButtonBuffer2[i]) && (debounceButtonBuffer1[i] == debounceButtonBuffer3[i])) {
			if (buttonBuffer[i] == BUTTON_IS_RELEASED && debounceButtonBuffer1[i] == BUTTON_IS_PRESSED) {
				button_flag[i] = 1;
			}
			buttonBuffer[i] = debounceButtonBuffer1[i];
		}
		if (buttonBuffer[i] == BUTTON_IS_PRESSED) {
			if (counterForButtonPress1s[i] < DURATION_FOR_AUTO_INCREASING) {
				counterForButtonPress1s[i]++;
			} else {
				flagForButtonPress1s[i] = 1;
			}
		} else {
			button_flag[i] = 0;
			counterForButtonPress1s[i] = 0;
			flagForButtonPress1s[i] = 0;
		}
	}
}

unsigned char is_button_pressed(uint8_t index) {
	if (index >= NO_OF_BUTTONS) return 0;
	return (buttonBuffer[index] == BUTTON_IS_PRESSED);
}

unsigned char is_button_pressed_1s(unsigned char index) {
	if (index >= NO_OF_BUTTONS) return 0xff;
	return (flagForButtonPress1s[index] == 1);
}
