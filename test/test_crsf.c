#include "unity.h"
#include "crsf.h"

void setUp(void){
    return;
}

void tearDown(void){
    return;
}


void test_crc8_standard_vector(void){
    char vector[] = "123456789";
    TEST_ASSERT_EQUAL_UINT8(0xBC, crc8((const uint8_t *)vector, 9));
}

void test_decode_valid_frame_all_max(void){
    uint8_t frame[26] = {0};

    frame[0] = 0xC8;
    frame[1] = 24;
    frame[2] = 0x16;

    for(int i = 3; i < 25; i++){
        frame[i] = 0xFF;
    }
    frame[25] = crc8(frame+2, 23);

    RC_Channels_t channels = {0};

    TEST_ASSERT_TRUE(decode_rc_frame(frame, sizeof(frame), &channels));

    TEST_ASSERT_EQUAL_UINT16(0x7FF, channels.ch[0]);
    TEST_ASSERT_EQUAL_UINT16(0x7FF, channels.ch[15]);


}

void test_invalid_sync_byte(void){
    uint8_t frame[26] = {0};

    frame[0] = 0xC4;
    frame[1] = 24;
    frame[2] = 0x16;

    for(int i = 3; i < 25; i++){
        frame[i] = 0xFF;
    }
    frame[25] = crc8(frame+2, 23);

    RC_Channels_t channels = {0};

    TEST_ASSERT_FALSE(decode_rc_frame(frame, sizeof(frame), &channels));
}


void test_invalid_frame_len(void){
    uint8_t frame[26] = {0};

    frame[0] = 0xC8;
    frame[1] = 20;
    frame[2] = 0x16;

    for(int i = 3; i < 25; i++){
        frame[i] = 0xFF;
    }
    frame[25] = crc8(frame+2, 23);

    RC_Channels_t channels = {0};

    TEST_ASSERT_FALSE(decode_rc_frame(frame, sizeof(frame), &channels));
}

void test_invalid_frame_type(void){
    uint8_t frame[26] = {0};

    frame[0] = 0xC8;
    frame[1] = 24;
    frame[2] = 0x08;

    for(int i = 3; i < 25; i++){
        frame[i] = 0xFF;
    }
    frame[25] = crc8(frame+2, 23);

    RC_Channels_t channels = {0};

    TEST_ASSERT_FALSE(decode_rc_frame(frame, sizeof(frame), &channels));

}

void test_detect_shifted_frame(void){
    uint8_t frame[30] = {0};

    frame[2] = 0xC8;
    frame[3] = 24;
    frame[4] = 0x16;

    for(int i = 5; i < 27; i++){
        frame[i] = 0xFF;
    }
    frame[27] = crc8(frame+4, 23);

    RC_Channels_t channels = {0};

    TEST_ASSERT_TRUE(decode_rc_frame(frame, sizeof(frame), &channels));
}

void test_invalid_crc(void){
    uint8_t frame[26] = {0};

    frame[0] = 0xC8;
    frame[1] = 24;
    frame[2] = 0x16;

    for(int i = 3; i < 25; i++){
        frame[i] = 0xFF;
    }
    frame[25] = crc8(frame+2, 23);

    frame[20] = 0xF1;

    RC_Channels_t channels = {0};

    TEST_ASSERT_FALSE(decode_rc_frame(frame, sizeof(frame), &channels));

}


void test_wrong_crc_byte(void){
    uint8_t frame[26] = {0};

    frame[0] = 0xC8;
    frame[1] = 24;
    frame[2] = 0x16;

    for(int i = 3; i < 25; i++){
        frame[i] = 0xFF;
    }
    frame[25] = crc8(frame+2, 23) ^ 0x01;

    RC_Channels_t channels = {0};

    TEST_ASSERT_FALSE(decode_rc_frame(frame, sizeof(frame), &channels));
}

void test_buffer_too_short(void){
    uint8_t frame[26] = {0};
    

    frame[0] = 0xC8;
    frame[1] = 24;
    frame[2] = 0x16;

    for(int i = 3; i < 25; i++){
        frame[i] = 0xFF;
    }
    frame[25] = crc8(frame+2, 23);

    RC_Channels_t channels = {0};

    TEST_ASSERT_FALSE(decode_rc_frame(frame, 25, &channels));
}

void test_only_ch0_max(void){
    uint8_t frame[26] = {0};

    frame[0] = 0xC8;
    frame[1] = 24;
    frame[2] = 0x16;

    frame[3] = 0xFF;
    frame[4] = 0x07;

    frame[25] = crc8(frame+2, 23);

    RC_Channels_t channels = {0};

    TEST_ASSERT_TRUE(decode_rc_frame(frame, sizeof(frame), &channels));

    TEST_ASSERT_EQUAL_UINT16(0x7FF, channels.ch[0]);
    for(int i = 1; i < 16; i++){
        TEST_ASSERT_EQUAL_UINT16(0, channels.ch[i]);
    }
}

void test_only_ch1_max(void){
    uint8_t frame[26] = {0};

    frame[0] = 0xC8;
    frame[1] = 24;
    frame[2] = 0x16;

    frame[4] = 0xF8;
    frame[5] = 0x3F;

    frame[25] = crc8(frame+2, 23);

    RC_Channels_t channels = {0};

    TEST_ASSERT_TRUE(decode_rc_frame(frame, sizeof(frame), &channels));

    TEST_ASSERT_EQUAL_UINT16(0, channels.ch[0]);
    TEST_ASSERT_EQUAL_UINT16(0x7FF, channels.ch[1]);
    for(int i = 2; i < 16; i++){
        TEST_ASSERT_EQUAL_UINT16(0, channels.ch[i]);
    }
}

void test_only_ch15_max(void){
    uint8_t frame[26] = {0};

    frame[0] = 0xC8;
    frame[1] = 24;
    frame[2] = 0x16;

    frame[23] = 0xE0;
    frame[24] = 0xFF;

    frame[25] = crc8(frame+2, 23);

    RC_Channels_t channels = {0};

    TEST_ASSERT_TRUE(decode_rc_frame(frame, sizeof(frame), &channels));

    for(int i = 0; i < 15; i++){
        TEST_ASSERT_EQUAL_UINT16(0, channels.ch[i]);
    }
    TEST_ASSERT_EQUAL_UINT16(0x7FF, channels.ch[15]);
}

void test_ch0_mid(void){
    uint8_t frame[26] = {0};

    frame[0] = 0xC8;
    frame[1] = 24;
    frame[2] = 0x16;

    frame[3] = 0xE0;
    frame[4] = 0x03;

    frame[25] = crc8(frame+2, 23);

    RC_Channels_t channels = {0};

    TEST_ASSERT_TRUE(decode_rc_frame(frame, sizeof(frame), &channels));

    TEST_ASSERT_EQUAL_UINT16(CRSF_CHANNEL_MID, channels.ch[0]);
    TEST_ASSERT_EQUAL_UINT16(0, channels.ch[1]);
}

void test_false_sync_before_frame(void){
    uint8_t frame[30] = {0};

    frame[0] = 0xC8;
    frame[1] = 24;

    frame[3] = 0xC8;
    frame[4] = 24;
    frame[5] = 0x16;

    for(int i = 6; i < 28; i++){
        frame[i] = 0xFF;
    }
    frame[28] = crc8(frame+5, 23);

    RC_Channels_t channels = {0};

    TEST_ASSERT_TRUE(decode_rc_frame(frame, sizeof(frame), &channels));

    TEST_ASSERT_EQUAL_UINT16(0x7FF, channels.ch[0]);
    TEST_ASSERT_EQUAL_UINT16(0x7FF, channels.ch[15]);
}


int main(void){

    UNITY_BEGIN();

    RUN_TEST(test_crc8_standard_vector);

    RUN_TEST(test_decode_valid_frame_all_max);

    RUN_TEST(test_invalid_sync_byte);

    RUN_TEST(test_invalid_frame_len);

    RUN_TEST(test_invalid_frame_type);

    RUN_TEST(test_detect_shifted_frame);

    RUN_TEST(test_invalid_crc);

    RUN_TEST(test_wrong_crc_byte);

    RUN_TEST(test_buffer_too_short);

    RUN_TEST(test_only_ch0_max);

    RUN_TEST(test_only_ch1_max);

    RUN_TEST(test_only_ch15_max);

    RUN_TEST(test_ch0_mid);

    RUN_TEST(test_false_sync_before_frame);

    return UNITY_END();
}