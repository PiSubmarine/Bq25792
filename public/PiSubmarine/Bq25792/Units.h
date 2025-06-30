#pragma once
#include <cstdint>

namespace PiSubmarine::Bq25792
{
	struct MilliVolts
	{
		int32_t Value;

		explicit constexpr MilliVolts(int32_t v) : Value(v) {}
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
		return MilliVolts(static_cast<int32_t>(v));
	}

	constexpr MilliVolts operator"" _mV(unsigned long long v)
	{
		return MilliVolts(static_cast<int32_t>(v));
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
		int32_t Value;

		explicit constexpr MilliAmperes(int32_t v) : Value(v) {}
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
		return MilliAmperes(static_cast<int32_t>(v));
	}

	constexpr MilliAmperes operator"" _mA(unsigned long long v)
	{
		return MilliAmperes(static_cast<int32_t>(v));
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

	struct Celcius
	{
		int32_t Halves; // Represents temperature in 0.5C units

		// Constructor from halves (number of 0.5C units)
		explicit constexpr Celcius(int32_t v) : Halves(v) {}

		constexpr float ToFloat() const
		{
			return Halves * 0.5f;
		}
	};

	constexpr Celcius operator"" _C(long double v)
	{
		// Multiply by 2 to store in 0.5C units, round to nearest int
		return Celcius(static_cast<int32_t>(v * 2.0 + 0.5));
	}

	constexpr Celcius operator"" _C(unsigned long long v)
	{
		return Celcius(static_cast<int32_t>(v * 2));
	}

}