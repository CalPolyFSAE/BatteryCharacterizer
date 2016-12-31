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
#include <LoggingFunctions.h>

const uint8_t MOSI = 3;
const uint8_t MISO = 4;
const uint8_t SCK = 5;
const uint8_t CS0 = 2;
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
#define Disc_Sense_Resistance   5 //Discharge sense resistance: 0.005, in fixed decimal format 3 places
#define Charge_Sense_Resistance 47 //charge sense resistance: 0.047. in fixed decimal format 3 places
#define vRef                    5 // reference voltage
#define vRef2FixedPoint			500 //reference voltage in 3 decimal point fixed math
#define vRef3FixedPoint			5000
#define rThermistor25			10000 // thermistor resistance at 25C
#define rSeries					10000 // series resistor
#define rSeriesFixedPoint		1000000 //fixed point version of above number
#define bThermistor				3971 //beta value of thermistor
#define RoomTemp                25
#define maxVoltage              5
#define MAX16B				65535 //max number of 16 bit number
#define MAX12B				4095 //See Above
#define MAX10B				1023 //See Above
#define MAX8B				255 //See Above
#define OVERCHARGEVOLTAGE	500	 //3 point fixed decimal representation of 0.5V
//voltage to overcharge to when targetting a voltage less than 4.2
//hopefully this will make an uint16_t num, but unsure
#define NUMSAMPLES				5 //not sure how many samples yet
#define SETCURRENT				// come up with an equation here: set current is 10A
#define F_CLK					16000000 //16MHz
#define GBL_CLK_PRESCALER		256
#define GBL_CLK_MAX_VAL			62500 //sets the clock to be 1s on CTC mode: F_Clk / GBL_CLK_PRESCALER
#define POLL_CLK_PRESCALER		256
const uint16_t WaitClkCutoff = 62; //sets clock to be 2 ms period or 500Hz: 0.001 * F_Clk / POLL_CLK_PRESCALER
const uint8_t CCDutyStartVal = 134; //52.4% duty cycle in 8b format. Calculated to produce a duty cycle that puts 10A through discharge circuit at 4.2V
const uint8_t CPDutyStartVal = 96; //37.4% duty cycle in  8b format. 30W at  4.2V
const uint32_t tCutoff = 246700; //need to calc this value, but it should be the resistive equivalent to 60C: 2467
//calculated by steinhart hart equation
//also in fixed point form

//status register for batteries, global so that it can be read by the logging ISR
#define WAITING		0 //waiting for counterpart battery
#define CHARGING	1 //charging to 4.2V or storage voltage
#define FULLCHARGED	2 //charged to 4.2V or something similar
#define CCIP		3 //Constant current discharge in progress
#define CCDONE		4 //finished constant current discharge
#define CPIP		5 //constant power discharge in progress
#define CPDONE		6 //finished constant power discharge
#define ALLDONE		7 //finished all tests and charge to storage voltage
//end status register

const float rinf = rThermistor25 * exp((-bThermistor / 25)); //this is a constant for calculating battery temperature

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

//Begin Global Variables
volatile uint32_t GblClk; //global clock

//Boolean as to whether or not to log data
volatile bool readTemp;
//End Global Variables

//Structures
struct Battery
{
	uint32_t Voltage; //3 point fixed decimal
	uint32_t Current; //3 point fixed decimal
	uint32_t Temperature; //2 point fixed decimal
	uint8_t Status; //current status of Battery
	uint16_t CPTime; //maybe would be nice if I didn't need this to reduce struct size
	uint8_t Name; //lets function know which structure has been passed to it
	bool Failed;
};

//Begin Function Definitions

uint16_t read_MCU_ADC(uint8_t T_Batt){ //using this instead of analogRead because analogRead uses some weird pin conversion stuff
	uint16_t ADCvoltage;
	ADMUX &= 11111000; //clearing mux
	ADMUX |= T_Batt; //setting muc channel to correct pin
	ADCSRA |= (1 << ADSC); //starting conversion
	while (!(ADCSRA & (1 << ADIF))); //waiting until interrupt flag triggers
	ADCSRA |= (1 << ADSC); //clearing interrupt flag(writing to flag resets flag)
	ADCvoltage = ADC; //returning ADC voltage

	return ADCvoltage;
}
void Waitfor(uint8_t millis)
{
	//wait for some time in milliseconds, basically delay()
	TCNT2 = 0;
	for (uint8_t i = 0; i < millis; i++)
	{
		while (TCNT2 <= WaitClkCutoff)
			; //wait
	}
}

uint32_t readBattTemp(uint8_t Batt)
{
	uint32_t vThermistor;
	vThermistor = (read_MCU_ADC(Batt) * vRef2FixedPoint) / MAX12B; //may need to ditch analogRead, also converting to Fixed math with 2 decimal points
	return (((rSeriesFixedPoint * vRef2FixedPoint) / vThermistor)
			- rSeriesFixedPoint); //returning resistance of Thermistor
}

uint32_t readCurrent(uint8_t Batt, uint32_t resistance)
{ //batt refers to which battery to enable
	uint32_t vBatt;
	vBatt = Transmit_SPI_Master_16(Batt << 3, 0, CS0); // bit shift left by 3 to move batt to the correct position in ADC register
	vBatt = (vBatt * vRef3FixedPoint) / MAX12B; //convert to fixed point 3 decimal places
	return (vBatt / resistance) * 1000; //resistance must  be in 3 point fixed decimal or it won't work right
										// multiplying by 1000 because  if both are in 3point fixed, then it stops being in 3 point fixed
										// can probably be fixed by creating a type that has the correct math, but that's beyond my skill level
										// and this needs to be done asap
}

uint32_t readVoltage(uint8_t Batt)
{ //batt refers to which battery enable

	uint32_t vBatt = Transmit_SPI_Master_16(Batt << 3, 0, CS0);
	return (vBatt * vRef3FixedPoint) / MAX12B; //scaled to 3 decimal place fixed point. Don't need any more decimal places because
											   // 12B can't do better than 1 mV, so just wasting it anyways
}

/* save this function if antialiasing issues arise due to battery capacitive enough of a load
 void avgVoltandCurr(uint16_t * avgVolt, uint16_t * avgCurr, uint8_t Batt, uint16_t resistance){
 uint32_t sumVolt, sumCurr;
 for (uint8_t i = 0; i < NUMSAMPLES; i ++){ //go for so many cycles
 //do some shit

 while(TCNT2 <= ); //wait for some time
 }
 *avgVolt = sumVolt / NUMSAMPLES;
 *avgCurr = sumCurr / NUMSAMPLES; //getting avg
 }
 */

struct Battery charge(struct Battery Batt, uint16_t vStop, uint16_t iStop)
{ // batt refers to which battery to enable. vstop refers to what voltage to start checking Istop at, or when to cutoff if vstop is not 4.2V. Istop refers to what current to stop charging at
//BattStatusPtr, points to the status register of the specific battery
	struct Battery Battcpy = Batt; //copy Batt
	if (Battcpy.Name == 0)
	{
		turnOffPWM(DISC_EN_1); // turn off PWM first
		PORTB |= (1 << CHARGE_EN_1) | (1 << CHARGE_1_1); //enable charger and charging mosfet
		PORTC &= ~(1 << CHARGE_2_1); //make sure not charging two batteries at once(could lead to short between batteries
		PORTC |= (1 << BATT_EN_1_1); //open connection to battery
									 //two separate battery enables to take advantage of body diodes, see EAGLE schematic
	}

	else if (Battcpy.Name == 1)
	{
		//same as above
		turnOffPWM(DISC_EN_1);
		PORTB |= (1 << CHARGE_EN_1);
		PORTB &= ~(1 << CHARGE_1_1);
		PORTC |= (1 << CHARGE_2_1);
		PORTD |= (1 << BATT_EN_2_1);

	}

	else if (Battcpy.Name == 2)
	{
		//same as above
		turnOffPWM(DISC_EN_2);
		PORTD |= (1 << CHARGE_EN_2) | (1 << CHARGE_1_2) | (1 << BATT_EN_1_2);
		PORTC &= ~(1 << CHARGE_2_2);

	}

	else if (Battcpy.Name == 3)
	{
		//same as above
		turnOffPWM(DISC_EN_2);
		PORTD |= (1 << CHARGE_EN_2);
		PORTC |= (1 << CHARGE_2_2) | (1 << BATT_EN_2_2);
		PORTD &= ~(1 << CHARGE_1_2);

	}
	//start charging
	Waitfor(2); //wait for 2ms
	Battcpy.Voltage = readVoltage(Battcpy.Name);
	if ((Battcpy.Voltage >= vStop + OVERCHARGEVOLTAGE) && (vStop != 4200)) //when vBatt is above
	{
		Battcpy.Status |= (1 << FULLCHARGED);
	}
	else if (Battcpy.Current <= iStop)
	{
		Battcpy.Status |= (1 << ALLDONE); //if it makes it here, then the battery is done charging.
		//A battery must have been CC and CP discharges and recharged to a storage voltage
	}
	return Battcpy;
}

struct Battery discharge10A(struct Battery Batt, uint8_t duty_cycle,
		uint8_t vChannel, uint8_t iChannel) //maybe switch duty_cycle to a pointer, so I don't have to keep making and destroying it
{
//i and vChannel are the ADC channels to read from, could use if statement, but would be slower, might work if separate function is used
	struct Battery Battcpy = Batt;
	enablePWM(Batt.Name); //make pwm is always enable, in case just got out of charging
	Battcpy.Voltage = readVoltage(vChannel);
	Battcpy.Current = readCurrent(iChannel, Disc_Sense_Resistance);

	for (uint8_t i = 0; i < 4; i++)
	{ //this number needs to be decided
		Battcpy.Voltage = readVoltage(vChannel);
		Battcpy.Current = readCurrent(iChannel, Disc_Sense_Resistance);
		if ((Battcpy.Voltage <= 2800) || (Battcpy.CCTime >= 5))
		{ //exit CC discharge if voltage gets too low. Cutoff is 2.8V
			turnOffPWM(Batt.Name); //shut it down
			if(Battcpy.Voltage <= 2800){
			Battcpy.Failed = true; //if the voltage gets this low, then something is wrong
			}
			break; //should break from loop
		}
		else if (Battcpy.CCTime >= 5){ //after 5 seconds, end the test
			turnOffPWM(Batt.Name);
			Battcpy.Status |= (1 << CCDONE);
			Battcpy.CCTime = 0;
		}

		else if (Battcpy.Current < 10000)
		{ //10A in 3 point fixed decimal, can be subbed with a variable if needed
			duty_cycle += 5; //magic number controlling how much change to the duty cycle is going to be needed
		}
		else if (Battcpy.Current > 10000)
		{
			duty_cycle -= 5; //see above
		}

		setDutyCycle(Battcpy.Name, duty_cycle);
		Waitfor(2); //wait for 2 ms, should be good. Might want to change wait to be triggered on clk pulse
	}
	return Battcpy;
}

struct Battery discharge30W(struct Battery Batt, uint16_t vStop,
		uint8_t * statusReg)
{ //batt refers to which battery to enable
	struct Battery Battcpy = Batt;
	uint32_t disc_en, power;
	Battcpy.Voltage = readVoltage(Battcpy.Name);
	Battcpy.Current = readCurrent(Battcpy.Name, Disc_Sense_Resistance);
	uint8_t duty_cycle;
	if (Battcpy.Voltage <= vStop)
	{
		turnOffPWM(Battcpy.Name); //turnoff pwm if done
		*statusReg |= (1 << CPDONE); // can I do this??? trying to edit a register given to me in the function
		if(Battcpy.Voltage <= vStop - 200){//if the battery is 200 mV less than what it should be, then shut down
											//don't know what to call this magic number
			Battcpy.Failed = true;
	}
	else
	{
		enablePWM(Battcpy.Name); //make sure that PWM is on
		for (uint8_t i = 0; i < 4; i++)
		{		//discharge batteries
			disc_en = 1;
			Battcpy.Voltage = readVoltage(Battcpy.Name);
			Battcpy.Current = readCurrent(Battcpy.Name, Disc_Sense_Resistance);
			power = (Battcpy.Voltage * Battcpy.Current) / 1000; //divided by 1000 to prevent it from being 6 point fixed decimal
			//because these numbers are 32 bit, we don't have to worry about overflow or anything
			if (power < 30 * 1000)
			{ //if less than 30W in 3 point fixed decimal, then increase duty cycle by ~2%
				duty_cycle += 5;
			}
			else if (power > 30 * 1000)
			{ //very simple P loop
				duty_cycle -= 5;
			}
			setDutyCycle(Battcpy.Name, duty_cycle);
			Waitfor(2); // wait for 2 ms
		}
	}

}
	return Battcpy;

}
uint16_t calcThermTemp(uint16_t rThermistor)
{
	float tThermistor;
	float rThermFloat = rThermistor / 1000;
	tThermistor = bThermistor / (log(rThermFloat / rinf));
	return tThermistor;

}


void Initialize_PWM(void) //correct values for each register still need to be determined
{
	DDRD |= (1 << DISC_EN_1) | (1 << DISC_EN_2); 			//enables PWM pins
	TCCR0A = 0b10100011; 		//timer set to fast pwm
	TCCR0B = 0b00000011; 			//timer clk = system clk / 64;
//outputs 16E6/64/255 = 980Hz PWM
//consider increasing this value
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

void configureGblClk(void)
{ //This function will set up the global clock

	TCCR1A = 0x00; //set counter1 as normal timer
	TCCR1B |= (1 << WGM12) | (1 << CS12); //set prescaler for clk/256 and operate on CTC mode
	OCR1A = GBL_CLK_MAX_VAL; //precalculated number
//OCR1A holds OCR1AL and OCR1AH
	TIMSK1 = 0x01; //set interrupt on overflow

}

void configureADCWaitClk(void)
{ // not sure exactly what to call this, but this loop dictates how long before taking an ADC measurement
	TCCR2A = 0x00; //set counter1 as normal timer
	TCCR2B |= (1 << CS22) | (1 << CS21); //set prescaler for 256

}

//
//void runforsometime(void) { //this is not a function, but a description of code to be inserted elsewhere
//	clear counter2
//	uint8_t randomvar, time;
//while((randomvar <= time))) { //need to check for longer 980Hz
//	//somecode				 //980Hz / 4 should be good
//	clearcounter
//	while(counteroverflowflag);//similar to delay function
//	randomvar ++;
//}
//}

//End Function Definitions

//Begin ISR definitions
ISR(TIMER1_OVF_vect)
{ //I'm not sure if this is how this works
//this functions works as a one second timer that causes the program to log data once a second

	GblClk++; //one second has passed so add some time
	readTemp = true;
}

//End ISR Definitions

//Begin Main
int main()
{
 //create batteries
struct Battery Batt0;
struct Battery Batt1;
struct Battery Batt2;
struct Battery Batt3;

//log data on startup
readTemp = true;

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
 //set up counters and clocks
configureGblClk();
configureADCWaitClk();
sei();
 //enable gbl interrupts
 //start with a few initial values
GblClk = 0; //initialize GblClk to be 0
Batt0.Voltage = readVoltage(Batt_Conn_1_1); //same for batt 1, 2, and 3
Batt1.Voltage = readVoltage(Batt_Conn_2_1);
Batt2.Voltage = readVoltage(Batt_Conn_1_2);
Batt3.Voltage = readVoltage(Batt_Conn_2_2);
Batt0.Temperature = readBattTemp(T_Batt_1);
Batt1.Temperature = readBattTemp(T_Batt_2);
Batt2.Temperature = readBattTemp(T_Batt_3);
Batt3.Temperature = readBattTemp(T_Batt_4); //start  by reading voltage and temperature through everything
Batt0.Current = 0;
Batt1.Current = 0;
Batt2.Current = 0;
Batt3.Current = 0;
Batt0.Failed = false;
Batt1.Failed = false;
Batt2.Failed = false;
Batt3.Failed = false;
Batt0.CCTime = 0;
Batt1.CCTime = 0;
Batt2.CCTime = 0;
Batt3.CCTime = 0;
Batt0.CPTime = 0;
Batt0.CPTime = 0;
Batt0.CPTime = 0;
Batt0.CPTime = 0;
Batt0.CPTime = 0;
 //set up variables for Discharging and charging batteries
uint8_t duty_cycle0 = 0, duty_cycle1 = 0; //duty_cycle0 controls the duty cycle for the discharge of batteries 0 and 1 duty_cycle1 controls batteries 2 and 3

while (1)
{ //main loop
  //batteries 0 and 1
if (!(Batt0.Failed || Batt1.Failed || Batt2.Failed || Batt3.Failed))//make sure none of the batteries failed
{
	if ((Batt0.Status & (1 << ALLDONE)) && (Batt1.Status & (1 << WAITING))) //if Batt0 is done testing and Batt1 is waiting to be tested, start testing
	{
//above bool states that if you're not doing a test, are fully charged, completely done testing and not waiting for another batteries test
		Batt1.Status &= ~(1 << WAITING); //stop Batt1 from waiting
		Batt0.Status |= (1 << WAITING); //make Batt0 to wait(ensuring that case where Batt0 and Batt1 are both not waiting doesn't happen
	}

	else if (!(Batt0.Status & (1 << WAITING))
			&& !(Batt0.Status & (1 << CCDONE)))
	{ //check to make sure not waiting on other battery
//Constant Current
		if (!(Batt0.Status & (1 << CCIP)))
		{
			duty_cycle0 = CCDutyStartVal; //start with a duty cycle close to what CC needs it to be
			Batt0.Status |= (1 << CCIP);
		}
		discharge10A (BATT0);
		if (duty_cycle0 == 0)
		{
			Batt0.Status |= (1 << CCDONE); //turn on CCDONE flag
			Batt0.Status &= ~(1 << CCIP); //turn off CCIP flag
		}
	}

	else if (!(Batt0.Status & (1 << WAITING))
			&& !(Batt0.Status & (1 << CPDONE)))
	{
//Constant Power Discharge
		if (!(Batt0.Status & (1 << CPIP)))
		{
			duty_cycle0 = CPDutyStartVal;
			Batt0.Status |= (1 << CPIP);
		}
		discharge30W (BATT0);
		if (duty_cycle0 == 0)
		{
			Batt0.Status |= (1 << CPDONE);
			Batt0.Status &= ~(1 << CPIP);
		}
	}

	else if ((Batt1.Status == 0) || (Batt1.Status & (1 << CHARGING)))
	{ //start charging or continue charging
//charging
		charge (BATT1);
	}

	else if (!(Batt1.Status & (1 << WAITING))
			&& !(Batt1.Status & (1 << CCDONE)))
	{ //check to make sure not waiting on other battery
//Constant Current
		if (!(Batt1.Status & (1 << CCIP)))
		{
			duty_cycle0 = CCDutyStartVal; //start with a duty cycle close to what CC needs it to be
			Batt1.Status |= (1 << CCIP);
		}
		discharge10A (BATT1);
		if (duty_cycle0 == 0)
		{
			Batt1.Status |= (1 << CCDONE); //turn on CCDONE flag
			Batt1.Status &= ~(1 << CCIP); //turn off CCIP flag
		}
	}

	else if (!(Batt1.Status & (1 << WAITING))
			&& !(Batt1.Status & (1 << CPDONE)))
	{
//Constant Power Discharge
		if (!(Batt1.Status & (1 << CPIP)))
		{
			duty_cycle0 = CPDutyStartVal;
			Batt1.Status |= (1 << CPIP);
		}
		discharge30W (BATT1);
		if (duty_cycle0 == 0)
		{
			Batt1.Status |= (1 << CPDONE);
			Batt1.Status &= ~(1 << CPIP);
		}
	}

//batteries 2 and 3
	if ((Batt2.Status == 0) || (Batt2.Status & (1 << CHARGING)))
	{ //start charging or continue charging
//charging
		charge (BATT2);
	}

	else if (!(Batt2.Status & (1 << WAITING))
			&& !(Batt2.Status & (1 << CCDONE)))
	{ //check to make sure not waiting on other battery
//Constant Current
		discharge10A (BATT2);
	}

	else if (!(Batt2.Status & (1 << WAITING))
			&& !(Batt2.Status & (1 << CPDONE)))
	{
//Constant Power Discharge
		discharge30W (BATT2);
	}

	else if ((Batt3.Status == 0) || (Batt3.Status & (1 << CHARGING)))
	{ //start charging or continue charging
//charging
		charge (BATT3);
	}

	else if (!(Batt3.Status & (1 << WAITING))
			&& !(Batt3.Status & (1 << CCDONE)))
	{ //check to make sure not waiting on other battery
//Constant Current
		discharge10A (BATT3);
	}

	else if (!(Batt3.Status & (1 << WAITING))
			&& !(Batt3.Status & (1 << CPDONE)))
	{
//Constant Power Discharge
		discharge30W (BATT3);
	}

	else
	{

	}
}
else
{
	//say failed or something
}
if (readTemp || (/*add code to force a log if a CC test finished. Don't care about CP test finishes*/))
{ //only measure temperature once a second
	Batt0.Temperature = readBattTemp(T_Batt_1);
	Batt1.Temperature = readBattTemp(T_Batt_2);
	Batt2.Temperature = readBattTemp(T_Batt_3);
	Batt3.Temperature = readBattTemp(T_Batt_4);
	if (Batt0.Temperature >= tCutoff)
	{
		Batt0.Failed = true; //end testing
	}
	if (Batt1.Temperature >= tCutoff)
	{
		Batt1.Failed = true; //end testing
	}
	if (Batt2.Temperature >= tCutoff)
	{
		Batt2.Failed = true; //end testing
	}
	if (Batt3.Temperature >= tCutoff)
	{
		Batt3.Failed = true; //end testing
	}

	//incrementing Time. As mentioned before, there's probably a better way to do this
	if(Batt0.Status & (1 << CCIP)){ //If you're in a CC test, increment this counter
		Batt0.CCTime ++;
	}
	else if (Batt0.Status & (1 << CPIP)){//If you're in a CP test, increment this counter
		Batt0.CPTime ++;
	}

	if(Batt1.Status & (1 << CCIP)){
		Batt1.CCTime ++;
	}
	else if (Batt1.Status & (1 << CPIP)){
		Batt1.CPTime ++;
	}

	if(Batt2.Status & (1 << CCIP)){
		Batt2.CCTime ++;
	}
	else if (Batt2.Status & (1 << CPIP)){
		Batt2.CPTime ++;
	}

	if(Batt3.Status & (1 << CCIP)){
		Batt3.CCTime ++;
	}
	else if (Batt3.Status & (1 << CPIP)){
		Batt3.CPTime ++;
	}
	logBattery(/*stuff*/); //this is where the battery data should be logged

	readTemp = false;

}
}
return 1;
}
