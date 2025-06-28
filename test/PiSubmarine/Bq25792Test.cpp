#include <gtest/gtest.h>
#include "PiSubmarine/Bq25792/Bq25792.h"

namespace PiSubmarine::Bq25792
{
	TEST(Bq25792, Reg00MinimalSystemVoltageTest)
	{
		constexpr std::array<uint8_t, 1> bytes{ 0b11111010 };
		Reg00MinimalSystemVoltage regR{ bytes };

		auto value = regR.GetMinimalSystemVoltage();
		ASSERT_EQ(value, 17000_mV);

		Reg00MinimalSystemVoltage regW;
		regW.SetMinimalSystemVoltage(3250_mV);
		ASSERT_EQ(regW.GetInternalBuffer()[0], 3);
	}
}