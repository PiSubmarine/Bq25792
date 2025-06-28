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
		ASSERT_EQ(regW.GetRegisterByteArray()[0], 3);
	}

	TEST(Bq25792, Reg03ChargeCurrentLimit)
	{
		constexpr std::array<uint8_t, 2> bytes{ 0x00, 101 };
		Reg03ChargeCurrentLimit reg{ bytes };

		auto value = reg.GetChargeCurrentLimit();
		ASSERT_EQ(value, 1010_mA);

		reg.SetChargeCurrentLimit(0_mA);
		ASSERT_EQ(reg.GetChargeCurrentLimit(), 0_mA);
		ASSERT_EQ(reg.GetRegisterByteArray()[0], 0);
		ASSERT_EQ(reg.GetRegisterByteArray()[1], 0);

		reg.SetChargeCurrentLimit(3250_mA);
		ASSERT_EQ(reg.GetChargeCurrentLimit(), 3250_mA);
		ASSERT_EQ(reg.GetRegisterByteArray()[0], 0x1);
		ASSERT_EQ(reg.GetRegisterByteArray()[1], 0x45);
	}
}