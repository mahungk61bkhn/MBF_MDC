/*
 * storage.c
 *
 *  Created on: May 30, 2023
 *      Author: Admin
 */
#include "storage.h"
#include "deviceFlash.h"
#include "stdlib.h"
#include "string.h"
#include "smc_gen/r_bsp/mcu/all/r_bsp_common.h"
#define FLASH_ADDR_SECTOR_CALIB_DATA 0x0



/*for external flash*/
uint16_t Mfr = 0, ID = 0;

storage_data_t storageData;

uint8_t data_write[100];
uint8_t data_read[100];
void storage_testFlash()
{
	for(int i = 0; i < 100; i++)
	{
		data_write[i] = 99 - i;
	}
	deviceFlash_erase4k(FLASH_ADDR_SECTOR_CALIB_DATA);
	deviceFlash_writeData(FLASH_ADDR_SECTOR_CALIB_DATA, data_write, 100);
	deviceFlash_readData(FLASH_ADDR_SECTOR_CALIB_DATA, data_read, 100);
	if(memcmp(data_read, data_write, 100) != 0)
	{
		while(1);
	}
}


uint32_t crc32_compute(uint8_t const * p_data, uint32_t size, uint32_t const * p_crc)
{
    uint32_t crc;

    crc = (p_crc == NULL) ? 0xFFFFFFFF : ~(*p_crc);
    for (uint32_t i = 0; i < size; i++)
    {
        crc = crc ^ p_data[i];
        for (uint32_t j = 8; j > 0; j--)
        {
            crc = (crc >> 1) ^ (0xEDB88320U & ((crc & 1) ? 0xFFFFFFFF : 0));
        }
    }
    return ~crc;
}


void storage_storeData()
{
	storageData.crc32 = crc32_compute(((uint8_t*)&storageData) + 4,sizeof(storageData)-4,NULL);
	deviceFlash_erase4k(FLASH_ADDR_SECTOR_CALIB_DATA);
	deviceFlash_writeData(FLASH_ADDR_SECTOR_CALIB_DATA,(uint8_t *)&storageData, sizeof(storageData));

	storage_data_t readback;
	deviceFlash_readData(FLASH_ADDR_SECTOR_CALIB_DATA,(uint8_t*)&readback,sizeof(readback));
	if(memcmp(&readback,&storageData,sizeof(storage_data_t)) != 0)
	{
		while(1);
	}

}
void storage_readDataStorage()
{
	R_BSP_SoftwareDelay(100, BSP_DELAY_MILLISECS);
	deviceFlash_readData(FLASH_ADDR_SECTOR_CALIB_DATA,(uint8_t*)&storageData,sizeof(storageData));
	uint32_t crc32 = crc32_compute(((uint8_t*)&storageData) + 4,sizeof(storageData)-4,NULL);
	if(crc32 != storageData.crc32)
	{
		memset(&storageData,0,sizeof(storageData));
//		storage_storeData();
	}
}

void storage_init()
{
	deviceFlash_readMfrID(&Mfr, &ID);

	storage_readDataStorage();
}

void storage_setCalibVoltage(float* cof, uint8_t num, bool isRestore)
{
	if(num > 12) return;
	for(int i = 0 ; i < num; i++)
	{
		storageData.valueFloat[i] = cof[i];
	}
	storageData.isCalib = isRestore ? 0: 1;
	storage_storeData();
}







