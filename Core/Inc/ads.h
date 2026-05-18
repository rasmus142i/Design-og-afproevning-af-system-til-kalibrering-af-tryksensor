/*
 * ads.h
 *
 *  Created on: Mar 19, 2026
 *      Author: THEM
 */

#ifndef INC_ADS_H_
#define INC_ADS_H_

#include "main.h"

#define CS_PORT GPIOB
#define CS_PIN  GPIO_PIN_6

#define START_PORT GPIOB
#define START_PIN  GPIO_PIN_8

#define DRDY_PORT GPIOB
#define DRDY_PIN GPIO_PIN_9

extern SPI_HandleTypeDef hspi1;

typedef struct {
	int minchan;
	int maxchan;
	int flag0; // always print data or not
	int flag1; // use the dumb write (1) or the smart write (0)
	int flag11;
} ADS_Stats;

void printAllDataDumb(uint8_t minchannel, uint8_t maxchannel, uint8_t already_set);
void unlockSPI();
void printAllData(uint8_t minchannel, uint8_t maxchannel);
void initVOLT_new();
void initTEMPx_new (uint8_t channel);
void initSPI();
void whileSPI();
void whileSPInew(uint8_t databuffer[6]);
uint8_t readRegSPI(uint8_t target);
void writeRegSPI(uint8_t target, uint8_t data);
void startSPI();
void readdataSPI(uint8_t outputbuffer[6]);
uint8_t _DRDY_pin_is_(GPIO_PinState state);
void sendSPI(uint8_t *command, uint8_t *pData, uint16_t length);
uint8_t _DRDY_bit_is_(uint8_t target);
int WriteLongSPI(uint8_t port, uint8_t data);

#endif /* INC_ADS_H_ */
