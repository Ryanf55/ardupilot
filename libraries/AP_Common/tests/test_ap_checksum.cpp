#include <AP_gtest.h>

#include <AP_Common/AP_Checksum.h>

TEST(AP_Common, fletcher16)
{
    const uint8_t wikipedia_ex1[2] {0x01, 0x02};
    EXPECT_EQ(AP::Fletcher16(wikipedia_ex1, 2), 0x0403);

}
AP_GTEST_MAIN()