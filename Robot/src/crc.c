/*
 * crc8.c
 *
 *  Created on: Jun 4, 2023
 *      Author: Reilly
 */
#include "crc.h"

uint8_t lut[256] = {0};

void CRC_Init(uint8_t poly)
{
	for (int idx = 0; idx < 256; ++idx)
	{
		uint8_t crc = idx;
		for (int shift = 0; shift < 8; ++shift)
		{
			crc = (crc << 1) ^ ((crc & 0x80) ? poly : 0);
		}
		lut[idx] = crc & 0xff;
	}
}

uint8_t CRC_Calc(uint8_t *data, uint8_t len)
{
	uint8_t crc = 0;
	while (len--)
	{
		crc = lut[crc ^ *data++];
	}
	return crc;
}
