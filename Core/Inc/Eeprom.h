/*
 * Eeprom.h
 *
 *  Created on: 17. 11. 2022
 *      Author: František Pospíšil
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "main.h"

#define EEPROM_WREN		0x06		// write enable
#define EEPROM_WRDI		0x04		// write disable
#define EEPROM_RDSR		0x05		// read status register
#define EEPROM_READ		0x03		// read from memory array
#define EEPROM_WRITE	0x02		// write to memory array

#define EEPROM_WIP_FLAG		0x01	// write in progress WIP flag
#define EEPROM_PAGESIZE		32

/* Timeout kazde jednotlive SPI operace [ms]. Zapis stranky trva realne ~5 ms,
   takze 50 ms je s velkou rezervou. Zaroven musi cely pristup do EEPROM skoncit
   vyrazne driv nez timeout watchdogu (~1 s) - nejhorsi pripad pri zcela nemluvici
   pameti je zhruba 2 stranky * 4 timeouty = ~400 ms, coz se bezpecne vejde. */
#define EEPROM_TIMEOUT_MS	50

#define EEPROM_CS_HIGH()	LL_GPIO_SetOutputPin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin)
#define EEPROM_CS_LOW()		LL_GPIO_ResetOutputPin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin)

typedef enum {					// seznam vysledku operace
	EEPROM_STATUS_PENDING,
	EEPROM_STATUS_COMPLETE,
	EEPROM_STATUS_ERROR
} EepromOperations;

void EEPROM_SPI_INIT(SPI_HandleTypeDef * hspi);

EepromOperations EEPROM_SPI_WriteBuffer(uint8_t* pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite);
EepromOperations EEPROM_SPI_ReadBuffer(uint8_t* pBuffer, uint16_t ReadAddr, uint16_t NumByteToRead);

#endif /* INC_EEPROM_H_ */
