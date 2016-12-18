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

/** pin mappings:
 *
 * T_Batt_1 = PC5
 * T_Batt_2 = PC4
 * T_Batt_3 = PC3
 * T_Batt_4 = PC2
 * RX[UART] = PD0
 * TX[UART] = PD1
 * CS0[SPI] = PB2
 * MOSI[SPI] = PB3
 * MISO [SPI] = PB4
 * SCK [SPI] = PB5
 * CHARGE_1_1 = PB1
 * CHARGE_2_1 = PD6
 * CHARGE_EN_1 = PB0
 * BATT_EN_1_1 = ADC6
 * BATT_EN_2_1 = PD7
 * DISC_EN_1 = PD5
 * CHARGE_1_2 = PD2
 * CHARGE_2_2 = ADC7
 * CHARGE_EN_2 = PD4
 * BATT_EN_1_2 = PD3
 * BATT_EN_2_2 = PC0
 * DISC_EN_2 = PC1
 *
 */
uint32_t read_current(uint32_t batt){
	uint32_t current;
	return current;
}

uint32_t read_voltage(uint32_t batt){
	uint32_t voltage;
	return voltage;
}


uint32_t charge(uint32_t batt, uint32_t vStop){
	uint32_t x;
	return x;
}

uint32_t discharge_30A(uint32_t batt){

	uint32_t time;
	return time;
}
/** Needed Changes
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
uint32_t discharge_30W(uint32_t batt, uint32_t vStop){

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

int main(){

	uint32_t vBatt0, iBatt0, pBatt0, vBatt1, iBatt1, pBatt1;
	uint32_t vBatt2, iBatt2, pBatt2, vBatt3, iBatt3, pBatt3;
	pBatt0 = vBatt0 * iBatt0; // getting power of battery
	pBatt1 = vBatt1 * iBatt1;
	pBatt2 = vBatt2 * iBatt2;
	pBatt3 = vBatt3 * iBatt3;
}
