/*
 * crsf.h
 *
 *  Created on: Jun 4, 2023
 *      Author: Reilly
 */

#ifndef INC_CRSF_H_
#define INC_CRSF_H_

#include "crsf_protocol.h"
#include <stdbool.h>

#define CRSF_CRC_POLY 0xd5

void CRSF_Init();
void CRSF_BytesReceived(uint8_t *rxBuf, uint8_t rxBufLen);
bool CRSF_DecodePacket(const crsf_header_t *hdr);
void CRSF_DecodeChannelsPacket(crsf_channels_t *ch_data);

#endif /* INC_CRSF_H_ */
