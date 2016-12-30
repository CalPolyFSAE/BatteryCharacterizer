/** LoggingFunctions.h
 * This file will contain all of the logging functions to be used
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <avr/io.h> // avr pin mapping library
#include <arduino/HardwareSerial.cpp>

uint32_t sendData (uint8_t Batt, uint32_t Data){
	uint8_t c;
	uint32_t Received;
	for (uint8_t i = 0; i < 4; i ++){
	c = Data >> i * 8;
	Serial1.write(c); //I don't know how Serial1write works, but I may have to add serial read
	Received = Serial1.read() << ((4 - i) * 8); //send msgs MSB first with first message being MSB
												//this function here allows us to write another message
	}
	return Received;
}




