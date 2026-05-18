/*s
 * ads.c
 *
 *  Created on: Mar 25, 2026
 *      Author: THEM
 */

#include "stdint.h"
#include "stdio.h"
#include "ads.h"

#define USE_WAIT 0
#define SPI_DELAY_MS 10

#define SAFE_WRITE 0
#define CONVRT 0 // 0: continuous, 1: one-shot
#define PRINT_STATUS 1
#define RASMUS_WOMKY 10

#define IMUXaddr 0x0D
#define IDACaddr 0x0E
#define REFRaddr 0x06
#define INPMUXaddr 0x11
#define MODE1addr 0x3
#define MODE3addr 0x5
#define PGAaddr 0x10
#define MODE0addr 0x2

void printAllDataDumb(uint8_t minchannel, uint8_t maxchannel, uint8_t already_set) {
	uint8_t db[6] = {123,123,123,123,123,123};

	if (already_set != 1) {
		HAL_GPIO_WritePin(START_PORT, START_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
		uint8_t stopcommand[2] = {0x0A, 0};
		sendSPI(stopcommand, NULL, 2);
		uint8_t resetcommand[2] = {0x06, 0};
		sendSPI(resetcommand, NULL, 2);
		uint8_t MODE3data = 0b01000000;
		uint8_t MODE1data = 0b00000001;
		uint8_t MODE0data = 0b01011011; // 4800 SPS, sinc4
		writeRegSPI(MODE0addr, MODE0data);
		writeRegSPI(MODE1addr, MODE1data);
		writeRegSPI(MODE3addr, MODE3data);
		printf("-----reset everything-----");
	}

	// Volt
	//sendSPI(stopcommand, NULL, 2);
	uint8_t PGAdata = 0b10000000; // 16x gain
	uint8_t IMUXdata = 0xFF; // No Con
	uint8_t IDACdata = 0xFF; // No Con
	uint8_t REFRdata = 0b00010101; // AVDD-AVSS refr. (5V)
	uint8_t INPMUXdata = 0b00110100; //PT0
	//uint8_t INPMUXdata = 0b01010110; //PT1
	writeRegSPI(IDACaddr, IDACdata);
	writeRegSPI(IMUXaddr, IMUXdata);
	writeRegSPI(PGAaddr, PGAdata);
	writeRegSPI(REFRaddr, REFRdata);
	writeRegSPI(INPMUXaddr, INPMUXdata);

	whileSPInew(db);
	printf("V:%02X%02X%02X%02X%02X%02X\r\n", db[0],db[1],db[2],db[3],db[4],db[5]);

	// Temps
	IDACdata = 0b100; // 500 µA
	REFRdata = 0b00011010; // AIN0/AIN1 external (2k ohm precision resistor)
	//PGAdata = 0b00000000; // 0 gain
	writeRegSPI(REFRaddr, REFRdata);
	writeRegSPI(IDACaddr, IDACdata);
	//writeRegSPI(PGAaddr, PGAdata);
	for (int i = minchannel; i < maxchannel; i++) {
		//sendSPI(stopcommand, NULL, 2);

		uint8_t IMUXdata = 0b0010 + i; // 0b0010 for channel 0, 0b0011 for channel 1, etc.
		uint8_t INPMUXdata = 0b00110001 + (i << 4); // 0b0011 0001 for channel 0, 0b0100 0001 for channel 1, etc.
		writeRegSPI(IMUXaddr, IMUXdata);
		writeRegSPI(INPMUXaddr, INPMUXdata);
		whileSPInew(db);
		printf("T%i:%02X%02X%02X%02X%02X%02X\r\n", i, db[0],db[1],db[2],db[3],db[4],db[5]);
	}
}

void whileSPInew(uint8_t databuffer[6]) {
	HAL_GPIO_WritePin(START_PORT, START_PIN, GPIO_PIN_SET);
	int n = 0;
	uint8_t drdyhigh = _DRDY_pin_is_(GPIO_PIN_SET);
	//printf("Waiting 5s");
	if (USE_WAIT) {
		HAL_Delay(100);
	} else {
		while (drdyhigh) { // DRDY pin low = new data
			// Wait for DRDY to indicate data is ready
			//HAL_Delay(1);
			//printf("%i%i.", status, drdy);
			n++;
			if (n >= 1500) {
				printf("E:Data never arrived,");
				break;
			}
			HAL_Delay(1);
			drdyhigh = _DRDY_pin_is_(GPIO_PIN_SET);
		}
	}
	/*if (n>=30) {
			HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
			uint8_t MODE3data = 0b11000000;
			writeRegSPI(MODE3addr, MODE3data);
			printf("-----powerdown everything-----\r\n");
			HAL_Delay(5000);
			MODE3data = 0b01000000;
			writeRegSPI(MODE3addr, MODE3data);
			printf("-----powerup -----\r\n");
	}*/
	printf("%i%i,", n, drdyhigh);
	readdataSPI(databuffer);
	HAL_GPIO_WritePin(START_PORT, START_PIN, GPIO_PIN_RESET);
}

void printAllData(uint8_t minchannel, uint8_t maxchannel) {
	uint8_t db[6] = {123,123,123,123,123};
	initVOLT_new();
	startSPI();
	whileSPInew(db);
	printf("V:%02X%02X%02X%02X%02X%02X\r\n", db[0],db[1],db[2],db[3],db[4],db[5]);

	for (int i = minchannel; i < maxchannel; i++) {
		initTEMPx_new(i);
		startSPI();
		whileSPInew(db);
		printf("T%i:%02X%02X%02X%02X%02X%02X\r\n", i, db[0],db[1],db[2],db[3],db[4],db[5]);
	}
}


void initTEMPx_new (uint8_t channel) {

	uint8_t IMUXdata = 0b0010 + (1 * channel); // 0b0010 for channel 0, 0b0011 for channel 1, etc.
	uint8_t IDACdata = 0b100;
	uint8_t REFRdata = 0b00011010;
	uint8_t INPMUXdata = 0b00110001 + ((1 * channel) << 4); // 0b0011 0001 for channel 0, 0b0100 0001 for channel 1, etc.
	uint8_t MODE3data = 0b01000000;
	uint8_t PGAdata = 0b000;

	uint8_t stopcommand[2] = {0x0A, 0};
	sendSPI(stopcommand, NULL, 2);

	uint8_t resetcommand[2] = {0x06, 0};
	sendSPI(resetcommand, NULL, 2);

	if (SAFE_WRITE) {
		int status = 0;
		status += (WriteLongSPI(IDACaddr, IDACdata) << 0);
		status += (WriteLongSPI(IMUXaddr, IMUXdata) << 1);
		status += (WriteLongSPI(REFRaddr, REFRdata) << 2);
		status += (WriteLongSPI(INPMUXaddr, INPMUXdata) << 3);
		status += (WriteLongSPI(MODE3addr, MODE3data) << 4);
		status += (WriteLongSPI(PGAaddr, PGAdata) << 5);
		if (PRINT_STATUS) {printf("T%iS:%i\r\n", channel, status);}
	} else {
		writeRegSPI(IDACaddr, IDACdata);
		writeRegSPI(IMUXaddr, IMUXdata);
		writeRegSPI(REFRaddr, REFRdata);
		writeRegSPI(INPMUXaddr, INPMUXdata);
		writeRegSPI(MODE3addr, MODE3data);
		writeRegSPI(PGAaddr, PGAdata);
	}
}

void initSPI() {

	  uint8_t IMUXdata = 0x0000;
	  uint8_t IDACdata = 0b100;
	  uint8_t REFRdata = 0b00011111;
	  uint8_t INPMUXdata = 0b00010010;
	  uint8_t MODEdata = 0b01000000;
	  uint8_t PGAdata = 0b00000000;

	  uint8_t stopcommand[2] = {0x0A, 0};
	  sendSPI(stopcommand, NULL, 2);

	  uint8_t resetcommand[2] = {0x06, 0};
	  sendSPI(resetcommand, NULL, 2);

		int status = 0;
		status += (WriteLongSPI(IDACaddr, IDACdata) << 0);
		status += (WriteLongSPI(IMUXaddr, IMUXdata) << 1);
		status += (WriteLongSPI(REFRaddr, REFRdata) << 2);
		status += (WriteLongSPI(INPMUXaddr, INPMUXdata) << 3);
		status += (WriteLongSPI(MODE3addr, MODEdata) << 4);
		status += (WriteLongSPI(PGAaddr, PGAdata) << 5);

		if (PRINT_STATUS) {printf("%i\r\n", status);}
}

void initVOLT_new() {
		uint8_t IMUXdata = 0xFF;
		uint8_t IDACdata = 0xFF;
		uint8_t REFRdata = 0b00010101;
		uint8_t INPMUXdata = 0b00110100; //PT0
		//uint8_t INPMUXdata = 0b01010110; //PT1
		uint8_t MODEdata = 0b01000000;
		uint8_t PGAdata = 0b0;

		uint8_t stopcommand[2] = {0x0A, 0};
		sendSPI(stopcommand, NULL, 2);

		uint8_t resetcommand[2] = {0x06, 0};
		sendSPI(resetcommand, NULL, 2);

		if (SAFE_WRITE) {
			int status = 0;
			status += (WriteLongSPI(IDACaddr, IDACdata) << 0);
			status += (WriteLongSPI(IMUXaddr, IMUXdata) << 1);
			status += (WriteLongSPI(REFRaddr, REFRdata) << 2);
			status += (WriteLongSPI(INPMUXaddr, INPMUXdata) << 3);
			status += (WriteLongSPI(MODE3addr, MODEdata) << 4);
			status += (WriteLongSPI(PGAaddr, PGAdata) << 5);
			if (PRINT_STATUS) {printf("VS:%i\r\n", status);}
		} else {
			writeRegSPI(IDACaddr, IDACdata);
			writeRegSPI(IMUXaddr, IMUXdata);
			writeRegSPI(REFRaddr, REFRdata);
			writeRegSPI(INPMUXaddr, INPMUXdata);
			writeRegSPI(MODE3addr, MODEdata);
			writeRegSPI(PGAaddr, PGAdata);
		}
}

void whileSPI() {
	startSPI();
	int n = 0;
	//printf("Waiting 5s");
	if (USE_WAIT) {
		HAL_Delay(100);
	} else {
		uint8_t status = readRegSPI(0x01);
		uint8_t drdy = (status & 0b00000100) >> 2;
		while (drdy != 1) { // DRDY bit high = new data
			// Wait for DRDY to indicate data is ready
			HAL_Delay(1);
			printf("%i...",drdy);
			n++;
			status = readRegSPI(0x01);
			drdy = (status & 0b100) >> 2;
			if (n >= 1500) {
				printf("E:Data never arrived %i %i.", status, drdy);
				break;
			}
		}
	}
	uint8_t databuffer[6] = {123, 123, 123, 123, 123, 123};
	readdataSPI(databuffer);
	for(uint8_t j=0;j<6;j++) {printf("%02X ",databuffer[j]); }
	printf("\r\n");
	//printf("Loop finished, waiting... \r\n");
	//HAL_Delay(1000);
}


uint8_t readRegSPI(uint8_t target)
{
	uint8_t buffer[3] = {0};
	uint8_t command[3] = {0x20 | (target & 0x1F), 0x00, 0x00};
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, command, buffer, 3, SPI_DELAY_MS);
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
	HAL_Delay(RASMUS_WOMKY);

	//printf("\r\n !!! READ DEBUG: %02X %02X %02X !!! \r\n", buffer[0], buffer[1], buffer[2]);
	return buffer[2];
}

void writeRegSPI(uint8_t target, uint8_t data)
{
	uint8_t buffer[2] = {0};
	uint8_t command[2] = {0x40 | (target & 0x1F), data};
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, command, buffer, 2, SPI_DELAY_MS);
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
	HAL_Delay(RASMUS_WOMKY);
}

uint8_t _DRDY_bit_is_(uint8_t target) {
	uint8_t statusbyte = readRegSPI(0x01);
	return (((statusbyte & 0b100)>>2) == target);
}

uint8_t _DRDY_pin_is_(GPIO_PinState state)
// either GPIO_PIN_RESET or GPIO_PIN_SET
{
	return HAL_GPIO_ReadPin(DRDY_PORT, DRDY_PIN) == state;
}

void startSPI()
{
	uint8_t startcommand[2] = {0x08,0x00};
	uint8_t outputbuffer[2];
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, startcommand, outputbuffer, 2, SPI_DELAY_MS);
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
	HAL_Delay(RASMUS_WOMKY);
	/*if (outputbuffer[1] == startcommand[0]) {
		//printf("Start command sent \r\n");
	}*/
}

void unlockSPI()
{
	uint8_t startcommand[2] = {0xF5,0x00};
	uint8_t outputbuffer[2];
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, startcommand, outputbuffer, 2, SPI_DELAY_MS);
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
	if (outputbuffer[1] == startcommand[0]) {
		printf("Unlock command sent \r\n");
	}
}

void readdataSPI(uint8_t outputbuffer[6])
{
	uint8_t startcommand[6] = {0x12, 0,0,0,0,0};
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, startcommand, outputbuffer, 6, SPI_DELAY_MS);
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
}

void sendSPI(uint8_t *command, uint8_t *pData, uint16_t length)
{
    // 1. Pull CS Low to start communication
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);

    // 2. Transmit the command byte first
    HAL_SPI_TransmitReceive(&hspi1, command, pData, length, SPI_DELAY_MS);

    // 4. Pull CS High to end communication
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
}

int WriteLongSPI(uint8_t port, uint8_t data) {
	//printf("Writing...");
	writeRegSPI(port, data);
	//printf(" ... Done. Reading... ");
	uint8_t output = readRegSPI(port);
	//printf(" ... Done: %02X \r\n", output);
	if (output == data) {
	  //printf("SUCCES. \r\n");
		return 1;
	}
	else {
		return 0;
	}
}
