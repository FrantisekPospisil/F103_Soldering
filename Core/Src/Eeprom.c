/*
 * Eeprom.c
 *
 *  Created on: 17. 11. 2022
 *      Author: František Pospíšil
 *
 *  Ovladac SPI EEPROM rady 25LCxxx.
 *
 *  Vsechna cekani maji timeout (EEPROM_TIMEOUT_MS). Driv tu byly nekonecne
 *  smycky (cekani na WIP flag, na HAL_BUSY) a volani Error_Handler(), takze
 *  vadna pamet nebo rozhozene SPI dokazaly zablokovat celou stanici vcetne
 *  regulace topeni. Nyni se chyba jen ohlasi navratovym kodem a stanice bezi
 *  dal - main.c ji vypise jako "EEPROM read/write ERR".
 */

#include "Eeprom.h"

static SPI_HandleTypeDef *	EEPROM_SPI;

/* --- interni pomocne funkce --- */
static uint8_t			EEPROM_WaitSpiReady(void);
static EepromOperations	EEPROM_SendInstruction(uint8_t* instruction, uint8_t size);
static EepromOperations	EEPROM_WaitStandbyState(void);
static EepromOperations	EEPROM_WriteEnable(void);
static void				EEPROM_WriteDisable(void);
static EepromOperations	EEPROM_WritePage(uint8_t* pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite);

/**
 * @brief Init EEPROM SPI
 *
 * @param hspi Pointer to SPI struct handler
 */
void EEPROM_SPI_INIT(SPI_HandleTypeDef * hspi) {
	EEPROM_SPI = hspi;
}

/* Pocka, az bude SPI volne. Vraci 1 pri uspechu, 0 pri vyprseni casu. */
static uint8_t EEPROM_WaitSpiReady(void) {
	uint32_t t;

	for (t = 0; t < EEPROM_TIMEOUT_MS; t++) {
		if (EEPROM_SPI->State == HAL_SPI_STATE_READY) {
			return 1;
		}
		LL_mDelay( 1 );
	}
	return 0;
}

/* Odesle prikaz (pripadne i adresu). */
static EepromOperations EEPROM_SendInstruction(uint8_t *instruction, uint8_t size) {
	if (EEPROM_WaitSpiReady() == 0) {
		return EEPROM_STATUS_ERROR;
	}
	if (HAL_SPI_Transmit(EEPROM_SPI, instruction, (uint16_t)size, EEPROM_TIMEOUT_MS) != HAL_OK) {
		return EEPROM_STATUS_ERROR;
	}
	return EEPROM_STATUS_COMPLETE;
}

/* Pocka na dokonceni zapisu - cte status registr, dokud je nastaveny WIP flag.
   CS zustava po celou dobu dole, pamet pak status posila opakovane. */
static EepromOperations EEPROM_WaitStandbyState(void) {
	uint8_t		status[1]  = { 0x00 };
	uint8_t		command[1] = { EEPROM_RDSR };
	uint32_t	t;

	EEPROM_CS_LOW();

	if (EEPROM_SendInstruction(command, 1) != EEPROM_STATUS_COMPLETE) {
		EEPROM_CS_HIGH();
		return EEPROM_STATUS_ERROR;
	}

	for (t = 0; t < EEPROM_TIMEOUT_MS; t++) {
		if (HAL_SPI_Receive(EEPROM_SPI, status, 1, EEPROM_TIMEOUT_MS) != HAL_OK) {
			EEPROM_CS_HIGH();
			return EEPROM_STATUS_ERROR;
		}
		if ((status[0] & EEPROM_WIP_FLAG) == 0) {			// zapis dokoncen
			EEPROM_CS_HIGH();
			return EEPROM_STATUS_COMPLETE;
		}
		LL_mDelay( 1 );
	}

	EEPROM_CS_HIGH();										// pamet se neozvala vcas
	return EEPROM_STATUS_ERROR;
}

static EepromOperations EEPROM_WriteEnable(void) {
	uint8_t				command[1] = { EEPROM_WREN };
	EepromOperations	status;

	EEPROM_CS_LOW();
	status = EEPROM_SendInstruction(command, 1);
	EEPROM_CS_HIGH();

	return status;
}

static void EEPROM_WriteDisable(void) {
	uint8_t command[1] = { EEPROM_WRDI };

	EEPROM_CS_LOW();
	(void)EEPROM_SendInstruction(command, 1);				// pri chybe uz neni co zachranovat
	EEPROM_CS_HIGH();
}

/* Zapis do jedne stranky - nesmi prekrocit hranici EEPROM_PAGESIZE.
   O rozdeleni delsiho zapisu na stranky se stara EEPROM_SPI_WriteBuffer. */
static EepromOperations EEPROM_WritePage(uint8_t* pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite) {
	uint8_t				header[3];
	EepromOperations	status = EEPROM_STATUS_ERROR;

	if (NumByteToWrite == 0) {
		return EEPROM_STATUS_COMPLETE;
	}
	if (EEPROM_WaitSpiReady() == 0) {
		return EEPROM_STATUS_ERROR;
	}
	if (EEPROM_WriteEnable() != EEPROM_STATUS_COMPLETE) {
		return EEPROM_STATUS_ERROR;
	}

	header[0] = EEPROM_WRITE;
	header[1] = WriteAddr >> 8;
	header[2] = (uint8_t)WriteAddr;

	EEPROM_CS_LOW();
	if (EEPROM_SendInstruction(header, 3) == EEPROM_STATUS_COMPLETE) {
		if (HAL_SPI_Transmit(EEPROM_SPI, pBuffer, NumByteToWrite, EEPROM_TIMEOUT_MS) == HAL_OK) {
			status = EEPROM_STATUS_COMPLETE;
		}
	}
	EEPROM_CS_HIGH();

	if (status == EEPROM_STATUS_COMPLETE) {
		status = EEPROM_WaitStandbyState();					// pocka, az pamet zapis dokonci
	}
	EEPROM_WriteDisable();

	return status;
}

/* Zapis libovolne delky - rozdeli se podle hranic stranek. */
EepromOperations EEPROM_SPI_WriteBuffer(uint8_t* pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite) {
	uint16_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;
	uint16_t sEE_DataNum = 0;

	EepromOperations pageWriteStatus = EEPROM_STATUS_PENDING;

	Addr = WriteAddr % EEPROM_PAGESIZE;
	count = EEPROM_PAGESIZE - Addr;
	NumOfPage = NumByteToWrite / EEPROM_PAGESIZE;
	NumOfSingle = NumByteToWrite % EEPROM_PAGESIZE;

	if (Addr == 0) {										// zacatek presne na hranici stranky
		if (NumOfPage == 0) {
			sEE_DataNum = NumByteToWrite;
			pageWriteStatus = EEPROM_WritePage(pBuffer, WriteAddr, sEE_DataNum);
			if (pageWriteStatus != EEPROM_STATUS_COMPLETE) {
				return pageWriteStatus;
			}
		} else {
			while (NumOfPage--) {
				sEE_DataNum = EEPROM_PAGESIZE;
				pageWriteStatus = EEPROM_WritePage(pBuffer, WriteAddr, sEE_DataNum);
				if (pageWriteStatus != EEPROM_STATUS_COMPLETE) {
					return pageWriteStatus;
				}

				WriteAddr += EEPROM_PAGESIZE;
				pBuffer += EEPROM_PAGESIZE;
			}
			sEE_DataNum = NumOfSingle;
			pageWriteStatus = EEPROM_WritePage(pBuffer, WriteAddr, sEE_DataNum);
			if (pageWriteStatus != EEPROM_STATUS_COMPLETE) {
				return pageWriteStatus;
			}
		}
	} else {												// zacatek uprostred stranky
		if (NumOfPage == 0) {
			if (NumOfSingle > count) {						// presahne do dalsi stranky
				temp = NumOfSingle - count;
				sEE_DataNum = count;
				pageWriteStatus = EEPROM_WritePage(pBuffer, WriteAddr, sEE_DataNum);
				if (pageWriteStatus != EEPROM_STATUS_COMPLETE) {
					return pageWriteStatus;
				}
				WriteAddr += count;
				pBuffer += count;
				sEE_DataNum = temp;
				pageWriteStatus = EEPROM_WritePage(pBuffer, WriteAddr, sEE_DataNum);
			} else {
				sEE_DataNum = NumByteToWrite;
				pageWriteStatus = EEPROM_WritePage(pBuffer, WriteAddr, sEE_DataNum);
			}
			if (pageWriteStatus != EEPROM_STATUS_COMPLETE) {
				return pageWriteStatus;
			}
		} else {
			NumByteToWrite -= count;
			NumOfPage = NumByteToWrite / EEPROM_PAGESIZE;
			NumOfSingle = NumByteToWrite % EEPROM_PAGESIZE;
			sEE_DataNum = count;
			pageWriteStatus = EEPROM_WritePage(pBuffer, WriteAddr, sEE_DataNum);
			if (pageWriteStatus != EEPROM_STATUS_COMPLETE) {
				return pageWriteStatus;
			}
			WriteAddr += count;
			pBuffer += count;
			while (NumOfPage--) {
				sEE_DataNum = EEPROM_PAGESIZE;
				pageWriteStatus = EEPROM_WritePage(pBuffer, WriteAddr, sEE_DataNum);
				if (pageWriteStatus != EEPROM_STATUS_COMPLETE) {
					return pageWriteStatus;
				}
				WriteAddr += EEPROM_PAGESIZE;
				pBuffer += EEPROM_PAGESIZE;
			}
			if (NumOfSingle != 0) {
				sEE_DataNum = NumOfSingle;
				pageWriteStatus = EEPROM_WritePage(pBuffer, WriteAddr, sEE_DataNum);
				if (pageWriteStatus != EEPROM_STATUS_COMPLETE) {
					return pageWriteStatus;
				}
			}
		}
	}
	return EEPROM_STATUS_COMPLETE;
}

/* Cteni libovolne delky - pri cteni neni omezeni strankou. */
EepromOperations EEPROM_SPI_ReadBuffer(uint8_t* pBuffer, uint16_t ReadAddr, uint16_t NumByteToRead) {
	uint8_t header[3];

	if (NumByteToRead == 0) {
		return EEPROM_STATUS_COMPLETE;
	}
	if (EEPROM_WaitSpiReady() == 0) {
		return EEPROM_STATUS_ERROR;
	}

	header[0] = EEPROM_READ;
	header[1] = ReadAddr >> 8;
	header[2] = (uint8_t)ReadAddr;

	EEPROM_CS_LOW();

	if (EEPROM_SendInstruction(header, 3) != EEPROM_STATUS_COMPLETE) {
		EEPROM_CS_HIGH();
		return EEPROM_STATUS_ERROR;
	}
	if (HAL_SPI_Receive(EEPROM_SPI, pBuffer, NumByteToRead, EEPROM_TIMEOUT_MS) != HAL_OK) {
		EEPROM_CS_HIGH();
		return EEPROM_STATUS_ERROR;						// driv se vzdy vracelo COMPLETE
	}

	EEPROM_CS_HIGH();

	return EEPROM_STATUS_COMPLETE;
}
