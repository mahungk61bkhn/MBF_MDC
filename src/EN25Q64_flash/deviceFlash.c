/*
 * deviceFlash.c
 *
 *  Created on: Feb 28, 2022
 *      Author: tranq
 */
#include "deviceFlash.h"

#include "Config_RSPI0.h"
#include "string.h"
#include "stdbool.h"
#include "r_cg_macrodriver.h"
//#include "r_cg_macrodriver.h"
/* Definition of private data types go here */
#define writeEN         0x06    // Write enable instruction
#define writeDIS        0x04    // Write disable instruction
#define readDAT         0x03    // Read Data instruction
#define writePage       0x02    // Program the selected page instruction
#define sector4k        0x20    // Sector Erase instruction 4kB
#define block32k        0x52    // Half lock Erase instruction 32k-byte
#define block64k        0xD8    // Block Erase instruction 64k-byte
#define chipEra         0x60    // Chip Erase instruction
#define RDSR            0x05    // Read Status Register instruction
#define WRSR            0x01    // Write Status Register instruction
#define RMDID           0x90    // Manufacturer/Device ID
#define RDID            0x9F    // Read Manufacturer Identification
#define BUSY_FLAG       0x01    // Write operation in progress

#define  FLASH_SEL_INACTIVE    	(PORTC.PODR.BIT.B4 = 1)
#define  FLASH_SEL_ACTIVE 		(PORTC.PODR.BIT.B4 = 0)

static uint8_t WaitForWriteEnd(void)
{
  uint8_t FLASH_Status = 0;
  uint16_t arr[1];
  uint16_t ret[1];

  /* Select the SPI FLASH                                                                                   */
  FLASH_SEL_ACTIVE;

  /* Send "Read Status Register" instruction                                                                */
//  Spi_SendReceive(RDSR);
  arr[0] = RDSR;
  SPI_Send_Receive(arr, 1, ret);

  /* Loop as long as the busy flag is set                                                                   */
  do
  {
    /* Send a dummy byte to generate the clock to read the value of the status register                     */

	  arr[0] = 0;
//	  FLASH_Status = Spi_SendReceive(0);
	  SPI_Send_Receive(arr, 1, ret);
	  FLASH_Status = (uint8_t)ret[0];


  } while((FLASH_Status & BUSY_FLAG) == 1);

  /* Deselect the SPI FLASH                                                                                 */
  FLASH_SEL_INACTIVE;

  return FLASH_Status;
}
void deviceFlash_readMfrID(uint16_t *Mfr, uint16_t *ID)
{
	uint16_t arr[6] = {0x90, 0, 0, 0, 0, 0};
	uint16_t ret[6];
	FLASH_SEL_ACTIVE;
    SPI_Send_Receive(arr, (uint16_t) 6, ret);
    FLASH_SEL_INACTIVE;
    *Mfr = ret[4]; *ID = ret[5];
}

bool deviceFlash_readData(uint32_t add, uint8_t *rx_DATA, uint32_t len)
{

    uint8_t *rx = rx_DATA;
    uint16_t *p = malloc((sizeof(uint16_t))*60);
    uint16_t *arr = malloc((sizeof(uint16_t))*60);
    if ((p==NULL) || (arr==NULL))
	{
		return false;
	}
    arr[0] = 0x03;
	arr[1] = (add&0x00FF0000)>>16;
	arr[2] = (add&0x0000FF00)>>8;
	arr[3] = add&0x000000FF;
	FLASH_SEL_ACTIVE;
	SPI_Send_Receive(arr, 4 , p);
	while (len > 40)
	{
		SPI_Send_Receive(arr + 4, 40 , p);
		for (uint8_t i = 0; i< 40; i++)
		{
			(uint8_t)(*(rx++)) = (uint8_t)(*(p++));
		}
		p -= 40; len -= 40;
	}
	SPI_Send_Receive(arr + 4, (uint16_t) len, p);
	for (uint8_t i = 0; i < len; i++)
	{
		rx[i] = (uint8_t)p[i];
	}
	FLASH_SEL_INACTIVE;
    free(p); free(arr);
	return true;

}
static void deviceFlash_writeEnable(void)
{
	uint16_t *arr = malloc(sizeof(uint16_t));
	uint16_t *ret = malloc(sizeof(uint16_t));
	arr[0] = writeEN;
	FLASH_SEL_ACTIVE;
	SPI_Send_Receive(arr, 1, ret);
	FLASH_SEL_INACTIVE;
	free (arr); free(ret);

//  WaitForWriteEnd();
    // Chip Select (CS#) must be driven High after the last bit of the instruction sequence has been shifted in
}
static void deviceFlash_writeDisable(void)
{
	uint16_t *arr = malloc(sizeof(uint16_t));
	uint16_t *ret = malloc(sizeof(uint16_t));
	arr[0] = writeDIS;
	FLASH_SEL_ACTIVE;
	SPI_Send_Receive(arr, 1, ret);
	FLASH_SEL_INACTIVE;
	free (arr); free(ret);
//  WaitForWriteEnd();
}
void deviceFlash_writePage(uint32_t add, uint8_t *tx_DATA, uint32_t len)
{
//	uint16_t* arr =  malloc((sizeof(uint16_t))*(len + 4));
//    uint16_t *ret = malloc((sizeof(uint16_t))*(len + 4));
//    uint16_t *value = malloc((sizeof(uint16_t))*len);
//    arr[0] = writePage;
//    arr[1] = (add&0x00FF0000)>>16;
//    arr[2] = (add&0x0000FF00)>>8;
//    arr[3] = add&0x000000FF;
//    for (uint32_t i = 0; i < len; i++)
//    {
//    	value[i] = tx_DATA[i];
//    }
//
//    deviceFlash_writeEnable();
//
//    FLASH_SEL_ACTIVE;
//    SPI_Send_Receive(arr, 4, ret);
//    SPI_Send_Receive(value, len, ret);
//
//    FLASH_SEL_INACTIVE;
//    WaitForWriteEnd();
//
//    deviceFlash_writeDisable();
//    free(arr); free(ret); free(value);


	uint8_t *tx = tx_DATA;
    uint16_t* arr =  malloc((sizeof(uint16_t))*60);
    uint16_t *ret = malloc((sizeof(uint16_t))*60);
    uint16_t *value = malloc((sizeof(uint16_t))*60);
    arr[0] = writePage;
	arr[1] = (add&0x00FF0000)>>16;
	arr[2] = (add&0x0000FF00)>>8;
	arr[3] = add&0x000000FF;

	deviceFlash_writeEnable();
	FLASH_SEL_ACTIVE;
	SPI_Send_Receive(arr, 4, ret);
	while(len > 40)
	{
		for (uint8_t i = 0; i < 40; i++)
		{
			value[i] = *(tx++);
		}
		SPI_Send_Receive(value, 40, ret);
		len -= 40;
	}
	for (uint8_t i = 0; i < len; i++)
	{
		value[i] = tx[i];
	}
	SPI_Send_Receive(value, len, ret);

	FLASH_SEL_INACTIVE;

	WaitForWriteEnd();
	deviceFlash_writeDisable();

	free(arr); free(ret); free(value);


}

void deviceFlash_writeData(uint32_t add, void *tx_DATA, uint32_t len)
{
    uint32_t offsetAdd;
    uint32_t dataSize;
    uint32_t dataDone = 0;
    uint8_t *p;

    while(len)
    {
        offsetAdd = add%256;
        if(offsetAdd + len > 256) dataSize = 256 - offsetAdd;
        else dataSize = len;

        p = (uint8_t*)tx_DATA;
        deviceFlash_writePage(add, (uint8_t*)&p[dataDone], dataSize);
        add += dataSize;
        dataDone += dataSize;
        len -= dataSize;
    }
}
void deviceFlash_erase64k(uint32_t add)
{
    uint16_t arr[4];
    uint16_t *ret = malloc((sizeof(uint16_t))*4);
    if(ret == NULL) return;
    arr[0] = block64k;
    arr[1] = (add&0x00FF0000)>>16;
    arr[2] = (add&0x0000FF00)>>8;
    arr[3] = add&0x000000FF;

    deviceFlash_writeEnable();

    FLASH_SEL_ACTIVE;
    SPI_Send_Receive(arr, 4, ret);
    FLASH_SEL_INACTIVE;

    WaitForWriteEnd();

    deviceFlash_writeDisable();
    R_BSP_SoftwareDelay(600,BSP_DELAY_MILLISECS);
    free(ret);
}

/*********************************************************************************************************//**
 * @brief
 * @param
 * @retval
 ************************************************************************************************************/
void deviceFlash_erase4k(uint32_t add)
{
	uint16_t arr[4];
	uint16_t *ret = malloc((sizeof(uint16_t))*4);
	if(ret == NULL) return;
	arr[0] = sector4k;
	arr[1] = (add&0x00FF0000)>>16;
	arr[2] = (add&0x0000FF00)>>8;
	arr[3] = add&0x000000FF;

	 deviceFlash_writeEnable();

	FLASH_SEL_ACTIVE;
	SPI_Send_Receive(arr, 4, ret);
	FLASH_SEL_INACTIVE;
	WaitForWriteEnd();

	 deviceFlash_writeDisable();
	 R_BSP_SoftwareDelay(100,BSP_DELAY_MILLISECS);
	 free(ret);
}

/*********************************************************************************************************//**
 * @brief
 * @param
 * @retval
 ************************************************************************************************************/
void deviceFlash_erase32k(uint32_t add)
{
	uint16_t arr[4];
	uint16_t *ret = malloc((sizeof(uint16_t))*4);
	if(ret == NULL) return;
	arr[0] = block32k;
	arr[1] = (add&0x00FF0000)>>16;
	arr[2] = (add&0x0000FF00)>>8;
	arr[3] = add&0x000000FF;

	 deviceFlash_writeEnable();

	FLASH_SEL_ACTIVE;
	SPI_Send_Receive(arr, 4, ret);
	FLASH_SEL_INACTIVE;
	WaitForWriteEnd();

	deviceFlash_writeDisable();
	R_BSP_SoftwareDelay(500,BSP_DELAY_MILLISECS);
	free(ret);
}
