#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include <gtest/gtest.h>

#include "PiSubmarine/Bq25792/Device.h"

namespace PiSubmarine::Bq25792
{
	class FakeDriver : public I2C::Api::IDriver
	{
	public:
		explicit FakeDriver(std::array<uint8_t, 0x49>& registers) : m_Registers(registers)
		{
		}

		[[nodiscard]] PiSubmarine::Error::Api::Result<void> Write(
			const uint8_t deviceAddress,
			const std::span<const uint8_t> txData) override
		{
			EXPECT_EQ(deviceAddress, Device::Address);
			EXPECT_FALSE(txData.size() > 0 && txData.data() == nullptr);

			if (txData.size() == 1)
			{
				m_ReadOffset = txData[0];
				return {};
			}

			EXPECT_GT(txData.size(), 1U);

			const auto offset = static_cast<std::size_t>(txData[0]);
			std::copy_n(txData.begin() + 1, txData.size() - 1, m_Registers.begin() + offset);
			return {};
		}

		[[nodiscard]] PiSubmarine::Error::Api::Result<void> Read(
			const uint8_t deviceAddress,
			const std::span<uint8_t> rxData) override
		{
			EXPECT_EQ(deviceAddress, Device::Address);
			EXPECT_FALSE(rxData.size() > 0 && rxData.data() == nullptr);

			std::copy_n(m_Registers.begin() + m_ReadOffset, rxData.size(), rxData.begin());
			return {};
		}

		[[nodiscard]] PiSubmarine::Error::Api::Result<void> WriteRead(
			const uint8_t deviceAddress,
			const std::span<const uint8_t> txData,
			const std::span<uint8_t> rxData) override
		{
			auto writeResult = Write(deviceAddress, txData);
			if (!writeResult.has_value())
			{
				return std::unexpected(writeResult.error());
			}

			return Read(deviceAddress, rxData);
		}

	private:
		std::array<uint8_t, 0x49>& m_Registers;
		std::size_t m_ReadOffset = 0;
	};

	TEST(Bq25792Test, MinimalSystemVoltage)
	{
		std::array<uint8_t, 0x49> mockData = { 0x26, 0x6, 0x90, 0x0, 0x64, 0x24, 0x1, 0x2c, 0xc3, 0x5, 0xe3, 0x0, 0xdc, 0x4b, 0x3d, 0xa2, 0x85, 0x40, 0x0, 0x1, 0x16, 0xaa, 0xc0, 0x7a, 0x54, 0x0, 0x32, 0x20, 0x0, 0x1, 0x0, 0x0, 0x0, 0x0, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xff, 0xf9, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x3c, 0x6a, 0x3c, 0x77, 0x0, 0x0, 0x0, 0x22, 0x0, 0x0, 0x0, 0x0, 0x0, 0x8 };
		std::array<uint8_t, 0x49> mockDataWriteExpected = { 0x2F, 0x6, 0x90, 0x0, 0x64, 0x24, 0x1, 0x2c, 0xc3, 0x5, 0xe3, 0x0, 0xdc, 0x4b, 0x3d, 0xa2, 0x85, 0x40, 0x0, 0x1, 0x16, 0xaa, 0xc0, 0x7a, 0x54, 0x0, 0x32, 0x20, 0x0, 0x1, 0x0, 0x0, 0x0, 0x0, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xff, 0xf9, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x3c, 0x6a, 0x3c, 0x77, 0x0, 0x0, 0x0, 0x22, 0x0, 0x0, 0x0, 0x0, 0x0, 0x8 };

		FakeDriver driver{mockData};
		Device device{driver};

		auto vsysminRead = device.GetMinimalSystemVoltage();
		ASSERT_TRUE(vsysminRead.has_value());
		EXPECT_EQ(vsysminRead.value(), 12000_mV);

		const auto vsysminWrite = 14250_mV;
		auto writeResult = device.SetMinimalSystemVoltage(vsysminWrite);
		ASSERT_TRUE(writeResult.has_value());

		vsysminRead = device.GetMinimalSystemVoltage();
		ASSERT_TRUE(vsysminRead.has_value());
		EXPECT_EQ(vsysminRead.value(), vsysminWrite);

		EXPECT_EQ(mockData, mockDataWriteExpected);
	}
}
