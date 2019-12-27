
#if 0
#include "gtest/gtest.h"
#include "rev/ColorSensorV3.h"

TEST(ColorStrings, ColorStringsAreCorrect) {
    ASSERT_EQ(rev::ColorSensorV3::ColorToString(rev::ColorSensorV3::Color::red), "Red");
    ASSERT_EQ(rev::ColorSensorV3::ColorToString(rev::ColorSensorV3::Color::green), "Green");
    ASSERT_EQ(rev::ColorSensorV3::ColorToString(rev::ColorSensorV3::Color::blue), "Blue");
    ASSERT_EQ(rev::ColorSensorV3::ColorToString(rev::ColorSensorV3::Color::yellow), "Yellow");
    ASSERT_EQ(rev::ColorSensorV3::ColorToString(rev::ColorSensorV3::Color::unknown), "Unknown");
}
#endif