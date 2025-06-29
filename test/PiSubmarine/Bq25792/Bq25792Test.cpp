#include <gtest/gtest.h>
#include "PiSubmarine/Bq25792/Bq25792.h"
#include "I2CDriverMock.h"
#include <chrono>

using namespace std::chrono_literals;

namespace PiSubmarine::Bq25792
{
	TEST(Bq25792, MinimalSystemVoltageTest)
	{
		I2CDriverMock driver;
		Bq25792<I2CDriverMock> device{driver};

		device.Read();
		while (device.IsDirty())
		{
			std::this_thread::sleep_for(100ms);
		}

		MilliVolts vsysmin = device.GetMinimalSystemVoltage();
		ASSERT_EQ(vsysmin, 12000_mV);
	}
}