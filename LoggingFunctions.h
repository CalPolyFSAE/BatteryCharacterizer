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
	}
	return Received;
}

void logBattery(struct Battery Batt)
{ //sends data to computer via usb through ftdi chip with uart
	uint16_t voltage, current, temperature;
	float16(&voltage, (float(Batt.Voltage) / 1000));
	float16(&current, (float(Batt.Current) / 1000));
	float16(&temperature, float(Batt.Temperature));
	Serial1.write(voltage); //this is wrong, but need to get the point across
}
/**NOTE:I changed the bottom two things
 *
 */


/**Devin and any other people in Code team: There are a few things that I need you to help me with here:
 * 1. I'm not sure if the Serial functions will work at all. They are from the Arduino Library, but I'm still dubious as to whether
 * they work or not, please see if they will work, and if not, please implement them
 * 2. I need to log to be written out for the logBattery function: This is how it will work:
 * 		The function will send the voltage and current data from the battery type. Currently the data is in 3 point fixed decimal form.
 * 		I'm not sure of exactly what the name of it is, but everything is in mB and mA currently, so please divide it by 10 and send a
 * 		a decimal point or write code on the other side to put a decimal point in. You can use floats or put a "." in a string
 * 		it doesn't really matter
 * 3. You need to send the temperature data: This one is not necessary so ask me about it if there's still some time
 * 4. I need to when a test has completed and the relevant data for that. There is a status register in the battery struct
 * this data.
 * 5. This can be done either with a code to analyze the data on the computer side. I need the computer to take the USB data that's sent
 * by the FTDI 232RL chip, which should be relaying exactly what is send by UART. to be written to be analyzed. There is a program or
 * something by FTDI that helps pull the UART data I think, you can look it up, I don't know the specifics.
 * This is the analysis that needs to be done:
 * 		When the CPDONE flag goes high for a battery, I need the CPTime value to be stored and multiplied by 30
 * 		When the CCDONE flag goes high, I need to record the CCENDVoltage reading.
 * 	If you could look up how I can send a command or text through terminal to the chip, that would also help me a bunch
 */




