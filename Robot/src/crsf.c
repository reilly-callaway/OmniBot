/*
 * crsf.c
 *
 *  Created on: Jun 4, 2023
 *      Author: Reilly
 */

/*
 * Originally adapted from: https://github.com/CapnBry/CRServoF/blob/master/lib/CrsfSerial/CrsfSerial.cpp
 */

#include "crsf.h"
#include "crc.h"
#include <string.h>

#define PWM_MAX 2000
#define PWM_MIN 1000

uint16_t channels[CRSF_NUM_CHANNELS];

bool isLinkUp;

/******* WEAKLY DEFINED CALLBACK FUNCTIONS *******/
__attribute__((weak)) void CRSF_OnLinkUp()
{

}

__attribute__((weak)) void CRSF_OnPacketChannels(uint16_t* channels)
{

}

void shiftRxBuffer(uint8_t *rxBuf, uint8_t cnt, uint8_t rxBufLen)
{
	// Delete the whole thing, just set length to zero
	if (cnt >= rxBufLen)
	{
		rxBufLen = 0;
	}
	else
	{
		memmove(rxBuf, &rxBuf[cnt], rxBufLen - cnt + 1);
		rxBufLen -= cnt;
	}
}

void CRSF_Init()
{
	CRC_Init(CRSF_CRC_POLY);
}

void CRSF_BytesReceived(uint8_t *rxBuf, uint8_t rxBufLen)
{
	bool reprocess = false;

	do
	{
		reprocess = false;
		if (rxBufLen > 1)
		{
			uint8_t len = rxBuf[1];

			// Sanity check the length, ditch the earlist byte if invalid
			if (len < 3 || len > CRSF_MAX_PAYLOAD_LEN + 2)
			{
				shiftRxBuffer(rxBuf, 1, rxBufLen);
				rxBufLen -= 1;
				reprocess = true;
			}
			// Complete packet?
			else if (rxBufLen >= len + 2)
			{
				uint8_t crc = rxBuf[2 + len - 1];
				uint8_t calcCrc = CRC_Calc(&rxBuf[2], len-1);

				if (crc == calcCrc)
				{
					CRSF_DecodePacket((crsf_header_t *)rxBuf);
					shiftRxBuffer(rxBuf, len + 2, rxBufLen);
					rxBufLen -= len + 2;
					reprocess = true;
				}
				else
				{
					shiftRxBuffer(rxBuf, 1, rxBufLen);
					rxBufLen -= 1;
					reprocess = true;
				}
			}
		}
	} while(reprocess);
}

bool CRSF_DecodePacket(const crsf_header_t *hdr)
{
	bool valid = false;
	if (hdr->device_addr == CRSF_ADDRESS_FLIGHT_CONTROLLER)
	{
		valid = true; // TODO: Consider validating based on frame type too

		switch(hdr->type)
		{
		case CRSF_FRAMETYPE_GPS:
			break;
		case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
			CRSF_DecodeChannelsPacket((crsf_channels_t *)&hdr->data);
			break;
		case CRSF_FRAMETYPE_LINK_STATISTICS:
			break;
		default:
			break;

		}
	}

	return valid;
}

void CRSF_DecodeChannelsPacket(crsf_channels_t *ch_data)
{
    channels[0] = ch_data->ch0;
    channels[1] = ch_data->ch1;
    channels[2] = ch_data->ch2;
    channels[3] = ch_data->ch3;
    channels[4] = ch_data->ch4;
    channels[5] = ch_data->ch5;
    channels[6] = ch_data->ch6;
    channels[7] = ch_data->ch7;
    channels[8] = ch_data->ch8;
    channels[9] = ch_data->ch9;
    channels[10] = ch_data->ch10;
    channels[11] = ch_data->ch11;
    channels[12] = ch_data->ch12;
    channels[13] = ch_data->ch13;
    channels[14] = ch_data->ch14;
    channels[15] = ch_data->ch15;

    for (uint8_t i = 0; i < CRSF_NUM_CHANNELS; ++i)
	{
    	// Map from [CRSF_1000, CRSF_2000] to [PWM_MIN, PWM_MAX]
    	channels[i] = ((PWM_MAX - PWM_MIN) * (int32_t)(channels[i] - CRSF_CHANNEL_VALUE_1000) / CRSF_CHANNEL_VALUE_SPAN) + PWM_MIN;
	}

    if (!isLinkUp)
    {
    	// Weakly defined callback
    	CRSF_OnLinkUp();
    }

    isLinkUp = true;
    CRSF_OnPacketChannels(channels);
}



