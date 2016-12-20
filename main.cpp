/**
 * Project: Battery Characterizer
 * Author(s): Nick Mah and ???
 * Project Desc.: Code to characterize batteries: MCU charges
 * batteries to 3.6-3.8V, then does 30A CC discharge to determine
 * ESR then charges to 4.2V, then discharges at 30W CP discharge
 * monitoring current, voltage and time to get energy, then
 * recharges battery to 3.6-3.7V while writing data to values on
 * spreadsheet on computer that it's connected to.
 * File Desc.: Main file for Battery Characterizer
 * Date: 12/16/2016
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <avr/io.h>

/** pin mappings on MCU:
 *
 * T_Batt_1 = PC5 Temp for battery 1
 * T_Batt_2 = PC4 See above
 * T_Batt_3 = PC3 jumper wire to ADC7 cut existing trace
 * T_Batt_4 = PC2 jumper wire to ADC6 cut existing trace
 * RX[UART] = PD0 UART RX pin
 * TX[UART] = PD1 UART Tx pin
 * CS0[SPI] = PB2 Chip select for ADC
 * MOSI[SPI] = PB3 SPI MOSI
 * MISO [SPI] = PB4 SPI MISO
 * SCK [SPI] = PB5 SPI clock
 * CHARGE_1_1 = PB1 pull low to enable charging battery 1
 * CHARGE_2_1 = PD6 jumper wire to PC 1 pull low to enable charging of battery 2
 * CHARGE_EN_1 = PB0 push high to enable charging of either battery 1 or 2
 * BATT_EN_1_1 = ADC6 jumper wire to PC2 cut existing trace
 * BATT_EN_2_1 = PD7 pull low to enable charging or discharging of battery 2
 * DISC_EN_1 = PD5 PWM to control power/current through discharge load for battery 1 and 2
 * CHARGE_1_2 = PD2 same as  charge_1_1 but for battery 3
 * CHARGE_2_2 = ADC7 jumper wire to PC3 cut existing trace
 * CHARGE_EN_2 = PD4 same as Charge_En_1, but for battery 3 and 4
 * BATT_EN_1_2 = PD3 same as batt_en_1_1 but for battery 3
 * BATT_EN_2_2 = PC0 same as batt_en_1_1, but for battery 4
 * DISC_EN_2 = PC1 jumper wire to PD6 same as disc_en_2 but for battery 3 and 4
 *
 */
// NOTE: DISC_EN_2 and CHARGE_2_1 are swapped
// 		 T_Batt_3 and Charge_2_2 are swapped
//		 T_Batt_4 and  Batt_En_1_1
/** pin mapping on ADC:
 * Charge_Sense_1 = IN7 Current sense for charging of battery 1 and 2
 * Batt_Conn_1_1 = IN5 Voltage of battery 1
 * Batt_Conn_2_1 = IN6 Voltage of battery 2
 * I_Sense_1 = IN3  Current sense for discharging of battery 1 and 2
 * Charge_Sense_2 = IN0 Current sense for charging of battery 3 and 4
 * Batt_Conn_1_2 = IN1 Voltage of battery 3
 * Batt_Conn_2_2 = IN2 Voltage of battery 4
 * I_Sense_2 = IN4  Current sense of discharging of battery 3 and 4
 */
uint32_t readCurrent(uint8_t batt) { //batt refers to which battery to enable
	uint32_t current;
	//this should read the ADC
	return current;
}

uint32_t readVoltage(uint8_t batt) { //batt refers to which battery enable
	uint32_t voltage;
	return voltage;
}

uint32_t charge(uint8_t batt, uint32_t vStop, uint32_t iStop) { // batt refers to which battery to enable. vstop refers to what voltage to start checking Istop at, or when to cutoff if vstop is not 4.2V. Istop refers to what current to stop charging at
	uint32_t time, stop_val;
	if (vStop == 4.2) {
		stop_val = iStop;
	} else {
		stop_val = vStop + 0.5; //0.5 refers to voltage, needs to be scaled properly
	}
	return time;
}

uint32_t discharge15A(uint8_t batt, uint32_t vBatt, uint32_t iBatt) { //batt refers to which battery to enable

	uint32_t;
	return;
}
/** Needed Features/improvements
 * Might want to change the discharge functions so
 * we only have 1 function with a 30A CC and 30W CP
 * option on the parameters. Not sure how to
 * implement this though. Code for CC and CP are almost
 * identical. This is because there parameter for the
 * while loop for discharging. Also, need to figure out
 * how to write values outside of the function to report
 * voltage and current continuously. This isn't a
 * necessary feature, but it would be very nice to have.
 * Also, need to figure out how to set fuse bits, and do
 * some port mapping.
 */
uint32_t discharge30W(uint8_t batt, uint32_t vStop) { //batt refers to which battery to enable

	uint32_t voltage, current, disc_en;
	voltage = readVoltage(batt);
	current = readCurrent(batt);

	while (voltage >= vStop) {
		//discharge batteries
		disc_en = 1;
		voltage = readVoltage(batt);
		current = readCurrent(batt);
	}
	return;

}

void logBattery(uint8_t batt, uint32_t vBatt, uint32_t iBatt, uint32_t pBatt) { //sends data to computer via usb through ftdi chip with uart
	double voltage, current, power;
	voltage = double(vBatt) / 4096;
	current = double(iBatt) / 4096;
	power = voltage * current;
}

void Initialize_SPI_Master(void) //correct values for each register still need to be determined
		{
	SPCR = (0 << SPIE) | 		//No interrupts
			(1 << SPE) | 				//SPI enabled
			(0 << DORD) | 			//send MSB first
			(1 << MSTR) | 			//master
			(0 << CPOL) | 			//clock idles low
			(0 << CPHA) | 			//sample leading edge
			(0 << SPR1) | (0 << SPR0); //clock speed
	SPSR = (0 << SPIF) | 		//SPI interrupt flag
			(0 << WCOL) | 			//Write collision flag
			(0 << SPI2X); 			//Doubles SPI clock
	PORTB = 1 << SS;  		// make sure SS is high

}

void Transmit_SPI_Master(char Data) //Function for ATMega to write data to. Must be rewritten to read data from ADC
{
	PORTB = 0 << CS0;			// assert the slave select
	SPDR = Data;     			// Start transmission, send high byte first
	while (!(SPSR & (1<<SPIF)));// Wait (poll) for transmission complete
	PORTB = 1 << CS0;			// deassert the slave select
}

void Initialize_PWM(void) //correct values for each register still need to be determined
		{
	DDRD = 0xFF; 			//set port D as outputs
	TCCR0A = 0b10100011; 		//timer set to fast pwm
	TCCR0B = 3; 			//timer clk = system clk / 64;
	//outputs 16E6/64/255 = 980Hz PWM
	OCR0A = 50; 			//compare value => 20% duty cycle to PD6
	OCR0B = 191; 			//compare value => 75% duty cycle to PD5
}

void Initialize_ADC0(void) //correct values for each register still need to be determined
		{
	ADCSRA = 0x87;	//Turn On ADC and set prescaler (CLK/128)
	ADCSRB = 0x00;	//turn off autotrigger
	ADMUX = 0x00;    	//Set ADC channel ADC0
}

int main() {

	uint32_t vBatt0, iBatt0, pBatt0, vBatt1, iBatt1, pBatt1;
	uint32_t vBatt2, iBatt2, pBatt2, vBatt3, iBatt3, pBatt3;
	bool battTested0, battTested1, battTested2, battTested3; //boolean for whether or not the battery has been tested
	Initialize_ADC0();
	Initialize_PWM();
	Initialize_SPI_Master();
	vBatt0 = readVoltage(0); //same for batt 1, 2, and 3

}
