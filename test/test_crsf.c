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
    uint8_t frame[26];

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



int main(void){

    UNITY_BEGIN();

    RUN_TEST(test_crc8_standard_vector);

    RUN_TEST(test_decode_valid_frame_all_max);

    return UNITY_END();
}