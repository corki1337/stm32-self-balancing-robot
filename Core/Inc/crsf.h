#pragma once
#include <stdbool.h>
#include <stdint.h>



#define CRSF_CHANNEL_MIN        172
#define CRSF_CHANNEL_MID        992
#define CRSF_CHANNEL_MAX        1811
#define CRSF_CHANNEL_HALF_RANGE 819.f

typedef struct{
	uint16_t ch[16];
}RC_Channels_t;


uint8_t crc8(const uint8_t * ptr, uint8_t len);

bool decode_rc_frame(const uint8_t *frame, uint8_t len, RC_Channels_t *Channels);
