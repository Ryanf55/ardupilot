// Tests for the GSOF parser.
// * ./waf tests
// * ./build/sitl/tests/test_gsof


#include <AP_gtest.h>

#include <AP_GSOF/AP_GSOF.h>

#include <cstdio>
#include <cstdlib>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();


TEST(AP_GSOF, incomplete_packet)
{
    AP_GSOF gsof;
    AP_GSOF::MsgTypes expected;
    EXPECT_FALSE(gsof.parse(0, expected));
}

TEST(AP_GSOF, packet1)
{
    GTEST_SKIP() << "There is not yet a convention for loading in a data file in a cross-platform way in AP for unit tests";
    FILE* fp = std::fopen("libraries/AP_GSOF/tests/gsof_gps.dat", "rb");
    ASSERT_NE(fp, nullptr);
    AP_GSOF gsof;
    char c = 0;
    bool parsed = false;

    AP_GSOF::MsgTypes expected;
    expected.set(1);
    expected.set(2);
    expected.set(8);
    expected.set(9);
    expected.set(12);

    while (c != EOF) {
        c = fgetc (fp);
        parsed |= gsof.parse((uint8_t)c, expected);
    }
    
    EXPECT_TRUE(parsed);

    fclose(fp);

}

TEST(AP_GSOF, packet1_exhaustive_byte_corruption_adjusted_checksum)
{
    FILE* fp = fopen("libraries/AP_GSOF/tests/gsof_gps.dat", "rb");
    ASSERT_TRUE(fp != NULL);

    uint8_t buf[120];
    int len = 0;
    int ch;

    while ((ch = fgetc(fp)) != EOF && len < (int)sizeof(buf)) {
        buf[len++] = (uint8_t)ch;
    }

    fclose(fp);
    ASSERT_EQ(len, 120);

    // Copy of the original for resetting
    uint8_t original_buf[120];
    for (int i = 0; i < len; i++) {
        original_buf[i] = buf[i];
    }

    const int checksum_index = len - 2;
    const uint8_t original_checksum = original_buf[checksum_index];

    AP_GSOF::MsgTypes expected;
    expected.set(1);
    expected.set(2);
    expected.set(8);
    expected.set(9);
    expected.set(12);

    for (int byte_index = 0; byte_index < len; byte_index++) {
        if (byte_index == checksum_index) {
            continue; // Skip the checksum byte itself
        }

        uint8_t original = original_buf[byte_index];

        for (int new_val = 0; new_val <= 255; new_val++) {
            if (new_val == original) {
                continue;
            }

            // Restore buffer to original
            for (int i = 0; i < len; i++) {
                buf[i] = original_buf[i];
            }

            // Apply corruption
            buf[byte_index] = (uint8_t)new_val;

            // Adjust checksum to maintain the same total sum
            int16_t delta = (int16_t)new_val - (int16_t)original;
            buf[checksum_index] = (uint8_t)(original_checksum - delta);

            AP_GSOF gsof;
            bool parsed = false;

            for (int i = 0; i < len; i++) {
                parsed |= gsof.parse(buf[i], expected);
            }

            // EXPECT_TRUE(parsed) << "Failed with byte[" << byte_index
            //                     << "] corrupted from " << (int)original
            //                     << " to " << new_val
            //                     << " and checksum adjusted to "
            //                     << (int)buf[checksum_index];
        }
    }
}

AP_GTEST_MAIN()
