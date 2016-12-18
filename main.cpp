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
 * CHARGE_2_1 = PD6 pull low to enable charging of battery 2
 * CHARGE_EN_1 = PB0 push high to enable charging of either battery 1 or 2
 * BATT_EN_1_1 = ADC6 jumper wire to PC2 cut existing trace
 * BATT_EN_2_1 = PD7 pull low to enable charging or discharging of battery 2
 * DISC_EN_1 = PD5 PWM to control power/current through discharge load for battery 1 and 2
 * CHARGE_1_2 = PD2 same as  charge_1_1 but for battery 3
 * CHARGE_2_2 = ADC7 jumper wire to PC3 cut existing trace
 * CHARGE_EN_2 = PD4 same as Charge_En_1, but for battery 3 and 4
 * BATT_EN_1_2 = PD3 same as batt_en_1_1 but for battery 3
 * BATT_EN_2_2 = PC0 same as batt_en_1_1, but for battery 4
 * DISC_EN_2 = PC1 same as disc_en_2 but for battery 3 and  4
 *
 */

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
uint32_t read_current(uint8_t batt){//batt refers to which battery to enable
	uint32_t current;
	//this should read the ADC
	return current;
}

uint32_t read_voltage(uint8_t batt){//batt refers to which battery enable
	uint32_t voltage;
	return voltage;
}


uint32_t charge(uint8_t batt, uint32_t vStop, uint32_t iStop){// batt refers to which battery to enable. vstop refers to what voltage to start checking Istop at, or when to cutoff if vstop is not 4.2V. Istop refers to what current to stop charging at
	uint32_t x;
	return x;
}

uint32_t discharge_30A(uint8_t batt){ //batt refers to what voltage to cutoff at

	uint32_t time;
	return time;
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
uint32_t discharge_30W(uint8_t batt, uint32_t vStop){ //batt refers to which battery to enable

	uint32_t time, voltage, current, disc_en;
	voltage = read_voltage(batt);
	current = read_current(batt);
	while(voltage >= vStop){
		//discharge batteries
		disc_en = 0b1;
		voltage = read_voltage(batt);
		current = read_current(batt);
	}
	return time;

}

void log_battery(uint8_t batt){ //sends data to computer via usb through ftdi chip with uart
	return;
}

int main(){

//	uint32_t vBatt0, iBatt0, pBatt0, vBatt1, iBatt1, pBatt1;
//	uint32_t vBatt2, iBatt2, pBatt2, vBatt3, iBatt3, pBatt3;

}
