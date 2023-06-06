/*
 * crc.h
 *
 *  Created on: Jun 4, 2023
 *      Author: Reilly
 */

#ifndef INC_CRC_H_
#define INC_CRC_H_

#include "main.h"

void CRC_Init(uint8_t poly);
uint8_t CRC_Calc(uint8_t *data, uint8_t len);

#endif /* INC_CRC_H_ */
