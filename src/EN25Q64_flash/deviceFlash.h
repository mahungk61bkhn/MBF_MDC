/*
 * deviceFlash.h
 *
 *  Created on: Feb 28, 2022
 *      Author: tranq
 */

#ifndef APP_PROGRAMER_FLASH_DEVICEFLASH_H_
#define APP_PROGRAMER_FLASH_DEVICEFLASH_H_
#include "stdint.h"
#include "stdbool.h"
typedef enum
{
    STATE_INIT = 0,
    STATE_CMD,
    STATE_WAIT,
    STATE_DONE,
} erase_flash_state_t;

void deviceFlash_readMfrID(uint16_t * Mfr, uint16_t * ID);
void deviceFlash_erase64k(uint32_t add);
bool deviceFlash_readData(uint32_t add, uint8_t *rx_DATA, uint32_t len);
void deviceFlash_writeData(uint32_t add, void *tx_DATA, uint32_t len);
//void deviceFlash_readJEDEC(uint8_t* MfrId, uint8_t* memoryType, uint8_t* capacity);
void deviceFlash_eraseChip(void);
void deviceFlash_writePage(uint32_t add, uint8_t *tx_DATA, uint32_t len);
//erase_flash_state_t deviceFlash_eraseChipTask();
void deviceFlash_initErase();
#endif /* APP_PROGRAMER_FLASH_DEVICEFLASH_H_ */
