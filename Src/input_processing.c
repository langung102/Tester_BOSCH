/*
 * input_processing.c
 *
 *  Created on: Aug 18, 2023
 *      Author: minhl
 */

#include "input_reading.h"

enum ButtonState{BUTTON_RELEASE, BUTTON_PRESSED, BUTTON_PRESSED_MORE_THAN_1_SECOND, BUTTON_PRESSED_LONG, BUTTON_DOUBLE_PRESS};
enum ButtonState buttonState = BUTTON_RELEASE;

void fsm_for_input_processing(void) {
	switch (buttonState) {
		case BUTTON_RELEASE:
			if (is_button_pressed(0)) {
				buttonState = BUTTON_PRESSED;
			}
			break;
		case BUTTON_PRESSED:
			if (!is_button_pressed(0)) {
				buttonState = BUTTON_RELEASE;
			} else {
				if (is_button_pressed_1s(0)) {
					buttonState = BUTTON_PRESSED_MORE_THAN_1_SECOND;
				}
			}
			break;
		case BUTTON_PRESSED_MORE_THAN_1_SECOND:
			if (!is_button_pressed(0)) {
				buttonState = BUTTON_RELEASE;
			}
		default:
			break;
	}
}
