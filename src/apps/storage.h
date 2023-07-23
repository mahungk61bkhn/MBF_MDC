/*
 * storage.h
 *
 *  Created on: May 30, 2023
 *      Author: Admin
 */

#ifndef APPS_STORAGE_H_
#define APPS_STORAGE_H_

#include "stdint.h"
#include "stdbool.h"
#define MAX_VALUE_ITEM_STORAGE 32
typedef struct
{
	uint32_t crc32;
	float valueFloat[MAX_VALUE_ITEM_STORAGE];
	uint16_t valueInt[MAX_VALUE_ITEM_STORAGE];
	char productId[50];
	uint8_t  isCalib;
} storage_data_t;

typedef enum
{
	STORE_TYPE_COF_V1_GRID_A = 0,
	STORE_TYPE_COF_V1_GRID_B,
	STORE_TYPE_COF_V2_GRID_A,
	STORE_TYPE_COF_V2_GRID_B,
	STORE_TYPE_COF_V3_GRID_A,
	STORE_TYPE_COF_V3_GRID_B,

	STORE_TYPE_COF_V1_GEN_A,
	STORE_TYPE_COF_V1_GEN_B,
	STORE_TYPE_COF_V2_GEN_A,
	STORE_TYPE_COF_V2_GEN_B,
	STORE_TYPE_COF_V3_GEN_A,
	STORE_TYPE_COF_V3_GEN_B,
} value_float_idx_t;

void storage_init();
void storage_testFlash();
void deviceFlash_erase4k(uint32_t add);
void storage_setCalibVoltage(float* cof, uint8_t num, bool isRestore);
void storage_storeData();
#endif /* APPS_STORAGE_H_ */
