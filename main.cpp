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

uint32_t read_current(uint32_t batt){
	uint32_t current;
	return current;
}

uint32_t read_volatge(uint32_t batt){
	uint32_t voltage;
	return voltage;
}

uint32_t charge_3_65(){
	uint32_t x;
	return x;
}

uint32_t charge_4_2(){
	uint32_t x;
	return x;
}

uint32_t discharge_30A(){

	uint32_t time;
	return time;
}

uint32_t discharge_30W(){

	uint32_t time;
	return time;

}

int main(){

	uint32_t vBatt0, iBatt0, pBatt0, vBatt1, iBatt1, pBatt1;
	uint32_t vBatt2, iBatt2, pBatt2, vBatt3, iBatt3, pBatt3;
	pBatt0 = vBatt0 * iBatt0; -- getting power of battery
	pBatt1 = vBatt1 * iBatt1;
	pBatt2 = vBatt2 * iBatt2;
	pBatt3 = vBatt3 * iBatt3;

}
