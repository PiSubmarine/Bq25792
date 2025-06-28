#pragma once

#include <cstdint>

namespace PiSubmarine::Bq25792
{
	struct MilliVolts
	{
		uint16_t Value;

		explicit constexpr MilliVolts(uint16_t v) : Value(v) {}
	};

	constexpr bool operator==(MilliVolts lhs, MilliVolts rhs)
	{
		return lhs.Value == rhs.Value;
	}

	constexpr bool operator!=(MilliVolts lhs, MilliVolts rhs)
	{
		return lhs.Value != rhs.Value;
	}

	constexpr MilliVolts operator"" _mV(long double v)
	{
		return MilliVolts(static_cast<uint16_t>(v));
	}

	constexpr MilliVolts operator"" _mV(unsigned long long v)
	{
		return MilliVolts(static_cast<uint16_t>(v));
	}

	constexpr MilliVolts operator+(MilliVolts lhs, MilliVolts rhs)
	{
		return MilliVolts(lhs.Value + rhs.Value);
	}

	constexpr MilliVolts operator*(MilliVolts lhs, MilliVolts rhs)
	{
		return MilliVolts(lhs.Value * rhs.Value);
	}

	constexpr MilliVolts operator-(MilliVolts lhs, MilliVolts rhs)
	{
		return MilliVolts(lhs.Value - rhs.Value);
	}

	struct MilliAmperes
	{
		uint16_t Value;

		explicit constexpr MilliAmperes(uint16_t v) : Value(v) {}
	};

	constexpr bool operator==(MilliAmperes lhs, MilliAmperes rhs)
	{
		return lhs.Value == rhs.Value;
	}

	constexpr bool operator!=(MilliAmperes lhs, MilliAmperes rhs)
	{
		return lhs.Value != rhs.Value;
	}

	constexpr MilliAmperes operator"" _mA(long double v)
	{
		return MilliAmperes(static_cast<uint16_t>(v));
	}

	constexpr MilliAmperes operator"" _mA(unsigned long long v)
	{
		return MilliAmperes(static_cast<uint16_t>(v));
	}

	constexpr MilliAmperes operator+(MilliAmperes lhs, MilliAmperes rhs)
	{
		return MilliAmperes(lhs.Value + rhs.Value);
	}

	constexpr MilliAmperes operator*(MilliAmperes lhs, MilliAmperes rhs)
	{
		return MilliAmperes(lhs.Value * rhs.Value);
	}

	constexpr MilliAmperes operator-(MilliAmperes lhs, MilliAmperes rhs)
	{
		return MilliAmperes(lhs.Value - rhs.Value);
	}
}