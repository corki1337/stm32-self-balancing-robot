#pragma once
#include <stdbool.h>
#include <stdint.h>


typedef struct{
	uint16_t channel_01;
	uint16_t channel_02;
	uint16_t channel_03;
	uint16_t channel_04;
	uint16_t channel_05;
	uint16_t channel_06;
	uint16_t channel_07;
	uint16_t channel_08;
	uint16_t channel_09;
	uint16_t channel_10;
	uint16_t channel_11;
	uint16_t channel_12;
	uint16_t channel_13;
	uint16_t channel_14;
	uint16_t channel_15;
	uint16_t channel_16;
}RC_Channels_t;


uint8_t crc8(const uint8_t * ptr, uint8_t len);

bool decode_rc_frame(const uint8_t *frame, uint8_t len, RC_Channels_t *Channels);
