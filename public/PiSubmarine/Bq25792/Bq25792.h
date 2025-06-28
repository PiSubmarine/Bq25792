#pragma once

#include "PiSubmarine/RegUtils.h"
#include "PiSubmarine/Bq25792/Units.h"

namespace PiSubmarine::Bq25792
{
	enum class RegOffset: uint8_t
	{
		MinimalSystemVoltage = 0x00,
		ChargeVoltageLimit = 0x01,
		ChargeCurrentLimit = 0x03,
		InputVoltageLimit = 0x05,
		InputCurrentLimit = 0x06,
		PrechargeControl = 0x08,
		TerminationControl = 0x09,
		RechargeControl = 0x0A,
		VotgRegulation = 0x0B,
		IotgRegulation = 0x0D,
		TimerControl = 0x0E,
		ChargetControl0 = 0x0F,
		ChargerControl1 = 0x10,
		ChargerControl2 = 0x11,
		ChargerControl3 = 0x12,
		ChargerControl4 = 0x13,
		ChargerControl5 = 0x14,
		TemperatureControl = 0x016,
		NtcControl0 = 0x17,
		NtcControl1 = 0x18,
		IcoCurrentLimit = 0x19,
		ChargerStatus0 = 0x1B,
		ChargerStatus1 = 0x1C,
		ChargerStatus2 = 0x1D,
		ChargerStatus3 = 0x1E,
		ChargerStatus4 = 0x1F,
		FaultStatus0 = 0x20,
		FaultStatus1 = 0x21,
		ChargerFlag0 = 0x22,
		ChargerFlag1 = 0x23,
		ChargerFlag2 = 0x24,
		ChargerFlag3 = 0x25,
		FaultFlag0 = 0x26,
		FaultFlag1 = 0x27,
		ChargerMask0 = 0x28,
		ChargerMask1 = 0x29,
		ChargerMask2 = 0x2A,
		ChargerMask3 = 0x2B,
		FaultMask0 = 0x2C,
		FaultMask1 = 0x2D,
		AdcControl = 0x2E,
		AdcFunctionDisable0 = 0x2F,
		AdcFunctionDisable1 = 0x30,
		IbusAdc = 0x31,
		IbatAdc = 0x33,
		VbusAdc = 0x35,
		Vac1Adc = 0x37,
		Vac2Adc = 0x39,
		VbatAdc = 0x3B,
		VsysAdc = 0x3D,
		TsAdc = 0x3F,
		TdieAdc = 0x41,
		DpAdc = 0x43,
		DmAdc = 0x45,
		DpDmDriver = 0x47,
		PartInformation = 0x48
	};

	struct Reg00MinimalSystemVoltage : RegUtils::Register<RegOffset::MinimalSystemVoltage, 1>
	{
		using RegUtils::Register<RegOffset::MinimalSystemVoltage, 1>::Register;

		RegUtils::Field<uint8_t, 0, 6, RegUtils::Access::ReadWrite, GetSize()> VsysMin{ Data };

		/// <summary>
		/// Get minimal system voltage.
		/// </summary>
		/// <returns>Voltage in mV</returns>
		constexpr MilliVolts GetMinimalSystemVoltage() const
		{
			return 2500_mV + MilliVolts(VsysMin) * 250_mV;
		}

		/// <summary>
		/// Sets minimal system voltage. Range: 2500mV - 16000mV, bit step size: 250mV
		/// </summary>
		/// <param name="valueMv">Voltage in mV</param>
		void SetMinimalSystemVoltage(MilliVolts valueMv)
		{
			uint16_t value = (valueMv.Value - 2500) / 250;
			VsysMin.Set(value);
		}
	};

	struct Reg01ChargeVoltageLimit : RegUtils::Register<RegOffset::ChargeVoltageLimit, 2>
	{
		using RegUtils::Register<RegOffset::ChargeVoltageLimit, 2>::Register;

		RegUtils::Field<uint8_t, 0, 11, RegUtils::Access::ReadWrite, GetSize()> Vreg{ Data };

		/// <summary>
		/// Gets battery target voltage.
		/// </summary>
		/// <returns>Voltage in mV</returns>
		constexpr MilliVolts GetBatteryVoltageLimit() const
		{
			return MilliVolts(Vreg) * 10_mV;
		}

		/// <summary>
		/// Sets battery target voltage. Range: 3000mV - 18800mV, bit step size: 10mV
		/// </summary>
		/// <param name="valueMv">Voltage in mV</param>
		void SetBatteryVoltageLimit(MilliVolts valueMv)
		{
			uint16_t value = valueMv.Value / 10;
			Vreg.Set(value);
		}
	};

	struct Reg03ChargeCurrentLimit : RegUtils::Register<RegOffset::ChargeCurrentLimit, 2>
	{
		using RegUtils::Register<RegOffset::ChargeCurrentLimit, 2>::Register;

		RegUtils::Field<uint8_t, 0, 9, RegUtils::Access::ReadWrite, GetSize()> Ichg{ Data };

		/// <summary>
		/// Gets maxium charge current.
		/// </summary>
		/// <returns>Current in mA</returns>
		constexpr MilliAmperes GetChargeCurrentLimit() const
		{
			return MilliAmperes(Ichg) * 10_mA;
		}

		/// <summary>
		/// Sets maximum charge current. Range: 50mA - 5000mA, bit step size: 10mA.
		/// </summary>
		/// <param name="valueMa">Current in mA</param>
		void SetChargeCurrentLimit(MilliAmperes valueMa)
		{
			uint16_t value = valueMa.Value / 10;
			Ichg.Set(value);
		}
	};

	struct Reg05InputVoltageLimit : RegUtils::Register<RegOffset::InputVoltageLimit, 1>
	{
		using RegUtils::Register<RegOffset::InputVoltageLimit, 1>::Register;

		RegUtils::Field<uint8_t, 0, 8, RegUtils::Access::ReadWrite, GetSize()> Vindpm{ Data };

		/// <summary>
		/// Gets input voltage dynamic power management.
		/// </summary>
		/// <returns>Voltage in mV</returns>
		constexpr MilliVolts GetInputVoltageDynamicPowerManagement() const
		{
			return MilliVolts(Vindpm) * 100_mV;
		}

		/// <summary>
		/// Sets input voltage dynamic power management. Range: 3600mV - 22000mV, bit step size: 100mV.
		/// </summary>
		/// <param name="valueMa">Voltage in mV</param>
		void SetInputVoltageDynamicPowerManagement(MilliVolts valueMv)
		{
			uint16_t value = valueMv.Value / 100;
			Vindpm.Set(value);
		}
	};

	struct Reg06InputCurrentLimit : RegUtils::Register<RegOffset::InputCurrentLimit, 2>
	{
		using RegUtils::Register<RegOffset::InputCurrentLimit, 2>::Register;

		RegUtils::Field<uint8_t, 0, 9, RegUtils::Access::ReadWrite, GetSize()> Iindpm{ Data };

		/// <summary>
		/// Gets input voltage dynamic power management.
		/// </summary>
		/// <returns>Voltage in mV</returns>
		constexpr MilliAmperes GetInputCurrentDynamicPowerManagement() const
		{
			return MilliAmperes(Iindpm) * 10_mA;
		}

		/// <summary>
		/// Sets input current dynamic power management. Range: 100mA - 3300mA, bit step size: 10mA.
		/// </summary>
		/// <param name="valueMa">Current in mA</param>
		void SetInputCurrentDynamicPowerManagement(MilliVolts valueMa)
		{
			uint16_t value = valueMa.Value / 10;
			Iindpm.Set(value);
		}
	};
}