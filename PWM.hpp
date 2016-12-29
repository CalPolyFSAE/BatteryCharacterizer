/**PWM functions
 *
 */
#include <avr/io.h>



void setDutyCycle(uint8_t Channel, uint16_t duty_cycle) { //this function exists because analogWrite doesn't use OCR0x register. Or at least it doesn't look like it

	if (Channel == 5) {

		OCR0A = duty_cycle;
	}

	else if (Channel == 6) {

		OCR0B = duty_cycle;
	}

}

void turnOffPWM(uint8_t Channel) {

	if (Channel == PD5) {

		OCR0A = 0; //set duty cycle to 0
		TCCR0A &= ~((1 << COM0A1) | (1 << COM0A0)); //disable OCR0A
	}

	else if (Channel == PD6) {

		OCR0B = 0;
		TCCR0A &= ~((1 << COM0B1) | (1 << COM0B0)); //See Above comments
	}
}

void enablePWM(uint8_t Channel){

	if (Channel == PD5){
		TCCR0A |= (1 << COM0A1) | (1 << COM0A0);
	}

	else if (Channel == PD6){

		TCCR0A |= (1 << COM0B1) | (1 << COM0B0);

	}
}
