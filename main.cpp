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
#include <avr/io.h> // avr pin mapping library
#include <avr/interrupt.h> //include interrupt abilities
#include <arduino/float16.hpp> //include half precision floating points
#include <arduino/SPI.h>
#include <arduino/HardwareSerial.cpp>
#include <PWM.hpp>

const uint8_t MOSI = 3;
const uint8_t MISO = 4;
const uint8_t SCK = 5;
const uint8_t CS0 = 2;
const float rinf = rThermistor25 * pow(e, (-bThermistor / 25));
#define T_Batt_1                PC5
#define T_Batt_2                PC4
#define T_Batt_3                7 //ADC6
#define T_Batt_4                6 //ADC7
#define TX                      PD1
#define RX                      PD0
#define CHARGE_1_1              PB1
#define CHARGE_2_1              PC1
#define CHARGE_EN_1             PB0
#define BATT_EN_1_1             PC2
#define BATT_EN_2_1             PD7
#define DISC_EN_1               PD5
#define CHARGE_1_2              PD2
#define CHARGE_2_2              PC3
#define CHARGE_EN_2             PD4
#define BATT_EN_1_2             PD3
#define BATT_EN_2_2             PC0
#define DISC_EN_2               PD6
#define XTAL1                   6
#define XTAL2                   7
#define Charge_Sense_1          7
#define Charge_Sense_2          0
#define Batt_Conn_1_1           5
#define Batt_Conn_2_1           6
#define Batt_Conn_1_2           1
#define Batt_Conn_2_2           2
#define I_Sense_1               3
#define I_Sense_2               4
#define Disc_Sense_Resistance   5 //Discharge sense resistance: 0.005
#define Charge_Sense_Resistance 5 //charge sense resistance: 0.047
#define vRef                    5 // reference voltage
#define rThermistor25			10000 // thermistor resistance at 25C
#define rSeries					10000 // series resistor
#define bThermistor				3971 //beta value of thermistor
#define RoomTemp                25
#define maxVoltage              5
#define CEILING16B				65535 //max number of 16 bit number
#define CEILING12B				4095 //See Above
#define CEILING10B				1023 //See Above
#define CEILING8B				255 //See Above
#define OVERCHARGEVOLTAGE		(CEILING10B / vRef) * 0.5 //10B representation of 0.5V
														  //voltage to overcharge to when targetting a voltage less than 4.2
														  //hopefully this will make an uint16_t num, but unsure
#define SETCURRENT				// come up with an equation here: set current is 10A
volatile uint16_t ClkCounter; //define a counter
volatile uint32_t GblClk; //global clock
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
 * DISC_EN_2 = PC1 jumper wire to PD6 same as disc_en_1 but for battery 3 and 4
 *
 */
// NOTE: DISC_EN_2 and CHARGE_2_1 are swapped
// 		 T_Batt_3 and Charge_2_2 are swapped
//		 T_Batt_4 and  Batt_En_1_1 are swapped
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

//uint16_t read_MCU_ADC(uint8_t T_Batt){
//	uint16_t ADCvoltage;
//	ADMUX &= 11111000; //clearing mux
//	ADMUX |= T_Batt; //setting muc channel to correct pin
//	ADCSRA |= (1 << ADSC); //starting conversion
//	while (!(ADCSRA & (1 << ADIF))); //waiting until interrupt flag triggers
//	ADCSRA |= (1 << ADSC); //clearing interrupt flag(writing to flag resets flag)
//	ADCvoltage = (ADCH << 8) | ADCL; //returning ADC voltage
//
//	return ADCvoltage;
//}

/** More notes:
 * also convert timing math to be
 */

uint16_t readBattTemp(uint8_t Batt) {
	uint16_t vThermistor;
	vThermistor = analogRead(Batt);
	return rSeries * ((vRef / vThermistor) - 1); //returning resistance of Thermistor
												 //not returning temperature since requires float math
										 //will calculate actual temperature during logging
}

uint16_t readCurrent(uint8_t Batt, uint16_t resistance) { //batt refers to which battery to enable
	uint16_t current, voltage;
	voltage = Transmit_SPI_Master_16(Batt << 3, 0, CS0); // bit shift left by 3 to move batt to the correct position in ADC register
	current = voltage / resistance;
	return current;
}

uint16_t readVoltage(uint8_t Batt) { //batt refers to which battery enable

	uint16_t voltage;
	voltage = Transmit_SPI_Master_16(Batt << 3, 0, CS0);
	voltage *= CEILING16B / CEILING12B; //map to 16 bit
	return voltage;
}

uint32_t charge(uint8_t Batt, uint16_t vStop, uint16_t iStop,
		uint16_t prevElapsedTime) { // batt refers to which battery to enable. vstop refers to what voltage to start checking Istop at, or when to cutoff if vstop is not 4.2V. Istop refers to what current to stop charging at

	uint16_t time, stop_val, vBatt, iBatt, prevTime;
	time = prevElapsedTime;
	if(Batt == 0){
		turnOffPWM(DISC_EN_1); // turn off PWM first
		PORTB |= (1 << CHARGE_EN_1) | (1 << CHARGE_1_1); //enable charger and charging mosfet
		PORTC &= ~(1 << CHARGE_2_1); //make sure not charging two batteries at once(could lead to short between batteries
		PORTC |= (1 << BATT_EN_1_1); //open connection to battery
									 //two separate battery enables to take advantage of body diodes, see EAGLE schematic

	}

	else if (Batt == 1){
		//same as above
		turnOffPWM(DISC_EN_1);
		PORTB |= (1 << CHARGE_EN_1);
		PORTB &= ~(1 << CHARGE_1_1);
		PORTC |= (1 << CHARGE_2_1);
		PORTD |= (1 << BATT_EN_2_1);


	}

	else if (Batt == 2){
		//same as above
		turnOffPWM(DISC_EN_2);
		PORTD |= (1 << CHARGE_EN_2) | (1 << CHARGE_1_2) | (1 << BATT_EN_1_2);
		PORTC &= ~(1 << CHARGE_2_2);

	}

	else if(Batt == 3){
		//same as above
		turnOffPWM(DISC_EN_2);
		PORTD |= (1 << CHARGE_EN_2);
		PORTC |= (1 << CHARGE_2_2) | (1 << BATT_EN_2_2);
		PORTD &= ~(1 << CHARGE_1_2);


	}

	if (vStop == 4.2) {
		stop_val = iStop; // this refers to what value we should stop charging at. This does not read battery voltage, it is merely setting the stop point
	} else {
		stop_val = vStop + OVERCHARGEVOLTAGE; //0.5 refers to voltage, needs to be scaled properly
	}
	//start charging
	while (time < 5) {
		prevTime = TCNT1;

		time += (TCNT1 - prevTime);
		vBatt = readVoltage(Batt);
		if(vBatt >= stop_val){
			//stop charging
		}
	}
	//need to return bool of charge being done
	return time;
}

uint16_t discharge10A(uint8_t Batt, uint16_t discTime, uint16_t duty_cycle) { //batt refers to which battery to enable
//not sure what data size time should be yet
// chosen resistor may not be low enough to handle 15A CC discharge
// if so, will switch to 10A
	uint16_t vBatt, iBatt, Time, prevTime;
	enablePWM(Batt);
	vBatt = readVoltage(Batt);
	iBatt = readCurrent(Batt, Disc_Sense_Resistance);
	Time = discTime;

	while (Time < 5) { //time needs to be scaled voltage also
		prevTime = TCNT1; //counter register
		vBatt = readVoltage(Batt);
		iBatt = readCurrent(Batt, Disc_Sense_Resistance);
		if (vBatt <= 2.8) { //exit CC discharge if voltage gets too low
			return NULL;
		}

		if (iBatt < 10) { //need to scale current
			duty_cycle++;
		}
		else if (iBatt > 10) {
			duty_cycle--;
		}

		if (vBatt <= 2.8) {
			duty_cycle = 0;

		}
		setDutyCycle(Batt, duty_cycle);
		Time += (TCNT1 - prevTime); //need to account for overflow
	}

	return Time;
}
uint16_t discharge30W(uint8_t Batt, uint16_t vStop, uint16_t prevElapsedTime) { //batt refers to which battery to enable

	uint16_t voltage, current, disc_en, power, elapsedTime, prevTime;
	uint8_t duty_cycle;
	enablePWM(Batt);
	prevTime = TCNT1; //counter register
	while ((voltage >= vStop) && (Time < 5)) {
		//discharge batteries
		disc_en = 1;
		voltage = readVoltage(Batt);
		current = readCurrent(Batt, Disc_Sense_Resistance);
		power = voltage * current;

		if (power < 30) { //scale this value properly
			duty_cycle++;
		} else if (power > 30) { //very simple P loop
			duty_cycle--;
		}
		setDutyCycle(Batt, duty_cycle);
		elapsedTime += (TCNT1 - prevTime); //need to account for overflow
	}
	return elapsedTime;

}

uint16_t calcThermTemp(uint16_t rThermistor) {
	float tThermistor;
	tThermistor = bThermistor / (log(rThermistor / rinf)); //need math.h and a way to calculate rInf automatically
	return tThermistor;

}


void logBattery(uint8_t batt, uint16_t vBatt, uint16_t iBatt, uint16_t tBatt) { //sends data to computer via usb through ftdi chip with uart
	uint16_t voltage, current, temperature;
	float16(&voltage, (float(vBatt) * 5 / 4096));
	float16(&current, (float(iBatt) * 5 / 4096));
	float16(&temperature, float(calcThermTemp(tBatt)));
	uint16_t power = voltage * current;
	Serial1.write(voltage); //this is wrong, but need to get the point accross
}

void Initialize_PWM(void) //correct values for each register still need to be determined
		{
	DDRD |= (1 << DISC_EN_1) | (1 << DISC_EN_2); 			//enables PWM pins
	TCCR0A = 0b10100011; 		//timer set to fast pwm
	TCCR0B = 0b00000011; 			//timer clk = system clk / 64;
//outputs 16E6/64/255 = 980Hz PWM
}

void Initialize_ADCs(void) //correct values for each register still need to be determined
		{
	ADCSRA = 0x87;	//Turn On ADC and set prescaler (CLK/128)
	ADCSRB = 0x00;	//turn off autotrigger
	ADMUX = 0x05;  //Set ADC channel ADC5(t_batt_1), set compare voltage to AVcc
	DIDR0 = (1 << T_Batt_2) | (1 << T_Batt_1); // Turning off digital input for T_Batt_1 and 2
// T_Batt_3 and 4 do not require this as they are
// ADC6 and ADC7
}

void configureGblClk(void){ //This function will set up the global clock

	TCCR2A = 0x02; //set counter2 as normal timer
	TCCR2B = 0x07; //set prescaler for clk/1024
	TIMSK2 = 0x01; //set interrupt on overflow

}

void runforsometime(void){ //this is not a function, but a description of code to be inserted elsewhere
	clear counter2
	uint8_t randomvar, time;
	while((randomvar != time))){ //need to check for longer 980Hz
		//somecode				 //980Hz / 4 should be good
		clearcounter
		while(counteroverflowflag); //similar to delay function
		randomvar ++;
	}
}

ISR(TOV2_vect){ //I'm not sure if this is how this works
	//this functions works as a one second timer that causes the program to log data once a second
	//considering not making this an interrupt since it might mess up SPI
	ClkCounter ++;
	if (ClkCounter == somenum){ //set this to be whatever it needs to be 1s
		ClkCounter = 0; //reset counter
		GblClk ++; //one second has passed so add some time
		logBattery(some globals); //would like a clever way to log data here
	}
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

int main() {

	uint16_t vBatt0, iBatt0, vBatt1, iBatt1;
	uint16_t tBatt0, tBatt1, tBatt2, tBatt3;
	uint16_t vBatt2, iBatt2, vBatt3, iBatt3;
	bool battTested0 = 0, battTested1 = 0, battTested2 = 0, battTested3 = 0; //boolean for whether or not the battery has been tested

//Declaration of Outputs and Inputs
	DDRC |= (1 << BATT_EN_2_2) | (1 << DISC_EN_2) | (1 << BATT_EN_1_1)
			| (1 << CHARGE_2_2);
	DDRD &= ~(1 << RX);
	DDRB &= ~((1 << MISO) | (1 << XTAL1) | (1 << XTAL2));
//Initialization of Communication Protocols
	Initialize_ADCs();
	Initialize_PWM();
	Serial1.begin(57600); //Initialize UART1
	Initialize_SPI_Master(CS0);
//set up 1 second clock
	configureGblClk();
	sei(); //enable gbl interrupts

	vBatt0 = readVoltage(Batt_Conn_1_1); //same for batt 1, 2, and 3
	vBatt1 = readVoltage(Batt_Conn_2_1);
	vBatt2 = readVoltage(Batt_Conn_1_2);
	vBatt3 = readVoltage(Batt_Conn_2_2);
	tBatt0 = readBattTemp(T_Batt_1);
	tBatt1 = readBattTemp(T_Batt_2);
	tBatt2 = readBattTemp(T_Batt_3);
	tBatt3 = readBattTemp(T_Batt_4); //start  by reading voltage and temperature through everything

	while (1) { //main loop

		if (!battTested0) { //
			vBatt0 = readVoltage(0);
			iBatt0 = readCurrent(0);
		}

		else if (!battTested1) {
			vBatt1 = readVoltage(1);
			iBatt1 = readCurrent(1);
		}

		else {

		}

		if (!battTested2) {
			vBatt2 = readVoltage(2);
			iBatt2 = readCurrent(2);
		}

		else if (!battTested3) {
			vBatt3 = readVoltage(3);
			iBatt3 = readCurrent(3);
		}

		else {

		}
	}
}
