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
		ChargerControl0 = 0x0F,
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

		RegUtils::Field<uint16_t, 0, 11, RegUtils::Access::ReadWrite, GetSize()> Vreg{ Data };

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

		RegUtils::Field<uint16_t, 0, 9, RegUtils::Access::ReadWrite, GetSize()> Ichg{ Data };

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

		RegUtils::Field<uint16_t, 0, 9, RegUtils::Access::ReadWrite, GetSize()> Iindpm{ Data };

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

	struct Reg08PrechargeControl : RegUtils::Register<RegOffset::PrechargeControl, 1>
	{
		using RegUtils::Register<RegOffset::PrechargeControl, 1>::Register;

		enum class PrechargeVoltageThreholds : uint8_t
		{
			Percent15 = 0,
			Percent62_2 = 1,
			Percent66_7 = 2,
			Percent71_4 = 3
		};

		RegUtils::Field<PrechargeVoltageThreholds, 6, 2, RegUtils::Access::ReadWrite, GetSize()> VbatLowv{ Data };
		RegUtils::Field<uint8_t, 0, 6, RegUtils::Access::ReadWrite, GetSize()> Iprechg{ Data };
		
		/// <summary>
		/// Gets Precharge current limit. Range: 40mA - 2000mA, bit step size: 40mA.
		/// </summary>
		/// <returns></returns>
		constexpr MilliAmperes GetPrechargeCurrentLimit() const
		{
			return MilliAmperes(Iprechg.Get()) * 40_mA;
		}

		/// <summary>
		/// Sets Precharge current limit. Range: 40mA - 2000mA, bit step size: 40mA.
		/// </summary>
		/// <param name="valueMa">Current in mA</param>
		void SetPrechargeCurrentLimit(MilliAmperes valueMa)
		{
			uint16_t value = valueMa.Value / 40;
			Iprechg.Set(value);
		}
	};

	struct Reg09TerminationControl : RegUtils::Register<RegOffset::TerminationControl, 1>
	{
		using RegUtils::Register<RegOffset::TerminationControl, 1>::Register;

		RegUtils::Field<uint8_t, 6, 1, RegUtils::Access::ReadWrite, GetSize()> RegRst{ Data };
		RegUtils::Field<uint8_t, 0, 5, RegUtils::Access::ReadWrite, GetSize()> Iterm{ Data };

		constexpr MilliAmperes GetTerminationCurrent() const
		{
			return MilliAmperes(Iterm.Get()) * 40_mA;
		}

		/// <summary>
		/// Sets Termination current. Range: 40mA - 1000mA, bit step size: 40mA.
		/// </summary>
		/// <param name="valueMa"></param>
		void SetTerminationCurrent(MilliAmperes valueMa)
		{
			uint16_t value = valueMa.Value / 40;
			Iterm.Set(value);
		}
	};

	struct Reg0ARechargeControl : RegUtils::Register<RegOffset::RechargeControl, 1>
	{
		using RegUtils::Register<RegOffset::RechargeControl, 1>::Register;

		enum class BatteryRechargeDeglichTimes: uint8_t
		{
			Milliseconds64 = 0, 
			Milliseconds256 = 1,
			Milliseconds1024 = 2,
			Milliseconds2048 = 3
		};

		RegUtils::Field<uint8_t, 6, 2, RegUtils::Access::ReadWrite, GetSize()> Cell{ Data };
		RegUtils::Field<BatteryRechargeDeglichTimes, 4, 2, RegUtils::Access::ReadWrite, GetSize()> Trechg{ Data };
		RegUtils::Field<uint8_t, 0, 4, RegUtils::Access::ReadWrite, GetSize()> Vrechg{ Data };
		
		/// <summary>
		/// Gets number of cells.
		/// </summary>
		/// <returns></returns>
		constexpr uint8_t GetCells() const
		{
			return Cell.Get() + 1;
		}

		/// <summary>
		/// Sets number of cells.
		/// </summary>
		/// <param name="cells"></param>
		void SetCells(uint8_t cells)
		{
			Cell.Set(cells - 1);
		}

		constexpr MilliVolts GetRechargeVoltageOffset() const
		{
			return 50_mV + MilliVolts(Vrechg.Get()) * 50_mV;
		}

		/// <summary>
		/// Sets recharge voltage offset. Range: 50mV - 800mV, bit step size: 50mV.
		/// </summary>
		/// <param name="valueMv"></param>
		void SetRechargeVoltageOffset(MilliVolts valueMv)
		{
			uint8_t value = (valueMv.Value - 50) / 50;
		}
	};

	struct Reg0BVotgRegulation : RegUtils::Register<RegOffset::VotgRegulation, 2>
	{
		using RegUtils::Register<RegOffset::VotgRegulation, 2>::Register;

		RegUtils::Field<uint16_t, 0, 11, RegUtils::Access::ReadWrite, GetSize()> Votg{ Data };

		/// <summary>
		/// Get OTG regulation voltage.
		/// </summary>
		/// <returns>Voltage in mV</returns>
		constexpr MilliVolts GetOtgRegulationVoltage() const
		{
			return 2800_mV + MilliVolts(Votg) * 10_mV;
		}

		/// <summary>
		/// Set OTG regulation voltage. Range: 2800mV - 22000mV, bit step size: 10mV.
		/// </summary>
		/// <param name="valueMv">Voltage in mV</param>
		void SetOtgRegulationVoltage(MilliVolts valueMv)
		{
			uint16_t value = (valueMv.Value - 2800) / 10;
			Votg.Set(value);
		}
	};

	struct Reg0DIotgRegulation : RegUtils::Register<RegOffset::IotgRegulation, 1>
	{
		using RegUtils::Register<RegOffset::IotgRegulation, 1>::Register;

		enum class PrechargeSafetyTimerValues
		{
			Hours2, 
			Minutes30
		};

		RegUtils::Field<PrechargeSafetyTimerValues, 7, 1, RegUtils::Access::ReadWrite, GetSize()> PrechgTmr{ Data };
		RegUtils::Field<uint8_t, 0, 7, RegUtils::Access::ReadWrite, GetSize()> Iotg{ Data };

		constexpr MilliAmperes GetOtgCurrentLimit() const
		{
			return MilliAmperes(Iotg) * 40_mA;
		}

		/// <summary>
		/// Sets OTG current limit. Range: 120mA - 3320mA, bit step size: 40mA.
		/// </summary>
		/// <param name="valueMa"></param>
		void SetOtgCurrentLimit(MilliAmperes valueMa)
		{
			uint16_t value = (valueMa.Value) / 40;
			Iotg.Set(value);
		}
	};

	struct Reg0ETimerControl : RegUtils::Register<RegOffset::TimerControl, 1>
	{
		using RegUtils::Register<RegOffset::TimerControl, 1>::Register;

		enum class TopOffTimerValues: uint8_t
		{
			Disabled = 0,
			Mins15,
			Mins30,
			Mins45
		};

		enum class FastChargeTimerValues : uint8_t
		{
			Hours5,
			Hours8,
			Hours12,
			Hours24
		};

		RegUtils::Field<TopOffTimerValues, 6, 2, RegUtils::Access::ReadWrite, GetSize()> TopOffTimer{ Data };
		RegUtils::Field<uint8_t, 5, 1, RegUtils::Access::ReadWrite, GetSize()> EnTrichgTmr{ Data };
		RegUtils::Field<uint8_t, 4, 1, RegUtils::Access::ReadWrite, GetSize()> EnPrechgTmr{ Data };
		RegUtils::Field<uint8_t, 3, 1, RegUtils::Access::ReadWrite, GetSize()> EnChgTmr{ Data };
		RegUtils::Field<FastChargeTimerValues, 1, 2, RegUtils::Access::ReadWrite, GetSize()> ChgTmr{ Data };
		RegUtils::Field<uint8_t, 0, 1, RegUtils::Access::ReadWrite, GetSize()> Tmr2xEn{ Data };
	};

	struct Reg0FChargerControl0 : RegUtils::Register<RegOffset::ChargerControl0, 1>
	{
		using RegUtils::Register<RegOffset::ChargerControl0, 1>::Register;

		enum class Flags : uint8_t
		{
			EnTerm = (1 << 1),
			EnHiz = (1 << 2),
			ForceIco = (1 << 3),
			EnIco = (1 << 4),
			EnChg = (1 << 5),
			ForceIbatDis = (1 << 6),
			EnAutoIbatDis = (1 << 7)
		};

		RegUtils::Field<Flags, 0, 7, RegUtils::Access::ReadWrite, GetSize()> Flags0{ Data };
	};

	struct Reg10ChargerControl1 : RegUtils::Register<RegOffset::ChargerControl1, 1>
	{
		using RegUtils::Register<RegOffset::ChargerControl1, 1>::Register;

		enum class VacOvpThresholds : uint8_t
		{
			V26 = 0,
			V18,
			V12,
			V7
		};

		enum class WatchdogValues : uint8_t
		{
			Sec0_5,
			Sec1,
			Sec2,
			Sec20,
			Sec40,
			Sec80,
			Sec160
		};

		RegUtils::Field<VacOvpThresholds, 4, 2, RegUtils::Access::ReadWrite, GetSize()> VacOvp{ Data };

		// Writing 1 resets Watchdog
		RegUtils::Field<uint8_t, 3, 1, RegUtils::Access::ReadWrite, GetSize()> WdRst{ Data };

		RegUtils::Field<WatchdogValues, 0, 3, RegUtils::Access::ReadWrite, GetSize()> Watchdog{ Data };
	};

	struct Reg11ChargerControl2 : RegUtils::Register<RegOffset::ChargerControl2, 1>
	{
		using RegUtils::Register<RegOffset::ChargerControl2, 1>::Register;

		enum class Flags : uint8_t
		{
			SdrvDly = (1 << 0),
			HvDcp = (1 << 3),
			En9V = (1 << 4),
			En12V = (1 >> 5),
			AutoIndetEn = (1 << 6),
			ForceIndet = (1 << 7)
		};

		enum class SdrvControlValues
		{
			Idle = 0,
			ShutdownMode,
			ShipMode,
			SystemPowerReset
		};

		RegUtils::Field<Flags, 0, 8, RegUtils::Access::ReadWrite, GetSize()> Flags2{ Data };
		RegUtils::Field<SdrvControlValues, 1, 3, RegUtils::Access::ReadWrite, GetSize()> SdrvCtrl{ Data };
	};


	struct Reg12ChargerControl3 : RegUtils::Register<RegOffset::ChargerControl3, 1>
	{
		using RegUtils::Register<RegOffset::ChargerControl3, 1>::Register;

		enum class Flags : uint8_t
		{
			DisFwdOoa = (1 << 0),
			DisOtgOoa = (1 << 1),
			DisLdo = (1 << 2),
			WkupDly = (1 << 3),
			PfmFwdDis = (1 << 4),
			PfmOtgDis = (1 << 5),
			EnOtg = (1 << 6),
			DisAcdrv = (1 << 7)
		};

		RegUtils::Field<Flags, 0, 8, RegUtils::Access::ReadWrite, GetSize()> Flags3{ Data };
	};

	struct Reg13ChargerControl4 : RegUtils::Register<RegOffset::ChargerControl4, 1>
	{
		using RegUtils::Register<RegOffset::ChargerControl4, 1>::Register;

		enum class Flags : uint8_t
		{
			EnIbusOcp = (1 << 0),
			ForceVinDpm = (1 << 1),
			DisVotgUvp = (1 << 2),
			DisVsysShort = (1 << 3),
			DisStat = (1 << 4),
			PwmFreq = (1 << 5),
			EnAcdrv1 = (1 < 6),
			EnAcdrv2 = (1 << 7)
		};

		RegUtils::Field<Flags, 0, 8, RegUtils::Access::ReadWrite, GetSize()> Flags4{ Data };
	};

	struct Reg14ChargerControl5 : RegUtils::Register<RegOffset::ChargerControl5, 1>
	{
		using RegUtils::Register<RegOffset::ChargerControl5, 1>::Register;

		enum class IbatRegValues
		{
			Amps3 = 0,
			Amps4 = 1,
			Amps5 = 2,
			Disable = 3
		};

		RegUtils::Field<uint8_t, 7, 1, RegUtils::Access::ReadWrite, GetSize()> SfetPresent{ Data };
		RegUtils::Field<uint8_t, 5, 1, RegUtils::Access::ReadWrite, GetSize()> EnIbat{ Data };
		RegUtils::Field<IbatRegValues, 3, 2, RegUtils::Access::ReadWrite, GetSize()> IbatReg{ Data };
		RegUtils::Field<uint8_t, 2, 1, RegUtils::Access::ReadWrite, GetSize()> EnIinDpm{ Data };
		RegUtils::Field<uint8_t, 1, 1, RegUtils::Access::ReadWrite, GetSize()> EnExtLim{ Data };
		RegUtils::Field<uint8_t, 0, 1, RegUtils::Access::ReadWrite, GetSize()> EnBatoc{ Data };
	};

	struct Reg16TemperatureControl : RegUtils::Register<RegOffset::TemperatureControl, 1>
	{
		using RegUtils::Register<RegOffset::TemperatureControl, 1>::Register;

		enum class ThermalRegulationThresholds
		{
			Deg60 = 0,
			Deg80 = 1,
			Deg100 = 2,
			Deg120 = 3
		};

		enum class ThermalShutdownThresholds
		{
			Deg150 = 0,
			Deg130 = 1,
			Deg120 = 2,
			Deg85 = 3
		};

		RegUtils::Field<ThermalRegulationThresholds, 6, 2, RegUtils::Access::ReadWrite, GetSize()> Treg{ Data };
		RegUtils::Field<ThermalShutdownThresholds, 4, 2, RegUtils::Access::ReadWrite, GetSize()> Tshut{ Data };
		RegUtils::Field<uint8_t, 3, 1, RegUtils::Access::ReadWrite, GetSize()> VbusPdEn{ Data };
		RegUtils::Field<uint8_t, 2, 1, RegUtils::Access::ReadWrite, GetSize()> Vac1PdEn{ Data };
		RegUtils::Field<uint8_t, 1, 1, RegUtils::Access::ReadWrite, GetSize()> Vac2PdEn{ Data };
	};

	struct Reg17NtcControl0 : RegUtils::Register<RegOffset::NtcControl0, 1>
	{
		using RegUtils::Register<RegOffset::NtcControl0, 1>::Register;

		enum class JeitaVset : uint8_t
		{
			ChargeSuspend = 0,
			Minus800mV,
			Minus600mV,
			Minus400mV,
			Minus300mV,
			Minus200mV,
			Minus100mV,
			VregUnchanged
		};

		enum class JeitaIset : uint8_t
		{
			ChargeSuspend = 0,
			Percent20,
			Percent40,
			IchgUnchanged
		};


		RegUtils::Field<JeitaVset, 5, 3, RegUtils::Access::ReadWrite, GetSize()> JeitaVset{ Data };
		RegUtils::Field<JeitaIset, 3, 2, RegUtils::Access::ReadWrite, GetSize()> JeitaSetHot{ Data };
		RegUtils::Field<JeitaIset, 1, 2, RegUtils::Access::ReadWrite, GetSize()> JeitaSetCold{ Data };
	};

	struct Reg18NtcControl1 : RegUtils::Register<RegOffset::NtcControl1, 1>
	{
		using RegUtils::Register<RegOffset::NtcControl1, 1>::Register;

		enum class TsCoolValues : uint8_t
		{
			Deg5,
			Deg10,
			Deg15,
			Deg20
		};

		enum class TsWarmValues : uint8_t
		{
			Deg40,
			Deg45,
			Deg50,
			Deg55
		};

		enum class BHotValues : uint8_t
		{
			Deg55,
			Deg60,
			Deg65,
			Disable
		};

		enum class BColdValues : uint8_t
		{
			DegMinus10,
			DegMinus20
		};

		RegUtils::Field<TsCoolValues, 6, 2, RegUtils::Access::ReadWrite, GetSize()> TsCool{ Data };
		RegUtils::Field<TsWarmValues, 4, 2, RegUtils::Access::ReadWrite, GetSize()> TsWarm{ Data };
		RegUtils::Field<BHotValues, 2, 2, RegUtils::Access::ReadWrite, GetSize()> BHot{ Data };
		RegUtils::Field<BColdValues, 1, 1, RegUtils::Access::ReadWrite, GetSize()> BCold{ Data };
		RegUtils::Field<uint8_t, 0, 1, RegUtils::Access::ReadWrite, GetSize()> TsIgnore{ Data };
	};

	struct Reg19IcoCurrentLimit : RegUtils::Register<RegOffset::IcoCurrentLimit, 2>
	{
		using RegUtils::Register<RegOffset::IcoCurrentLimit, 2>::Register;

		RegUtils::Field<uint16_t, 0, 9, RegUtils::Access::Read, GetSize()> IcoIlim{ Data };

		/// <summary>
		/// Get Input Current Limit obtained from ICO or ILIM_HIZ pin setting.
		/// </summary>
		/// <returns>Voltage in mV</returns>
		constexpr MilliAmperes GetIcoCurrentLimit() const
		{
			return MilliAmperes(IcoIlim) * 10_mA;
		}
	};

	struct Reg1BChargerStatus0 : RegUtils::Register<RegOffset::ChargerStatus0, 1>
	{
		using RegUtils::Register<RegOffset::ChargerStatus0, 1>::Register;

		enum class Flags : uint8_t
		{
			VbusPresentStat = (1 << 0),
			Ac1PresentStat = (1 << 1),
			Ac2PresentStat = (1 << 2),
			PgStat = (1 << 3),
			PoorSrcStat = (1 << 4),
			WdStat = (1 << 5),
			VinDpm = (1 << 6),
			IinDpmStat = (1 << 7)
		};

		RegUtils::Field<Flags, 0, 8, RegUtils::Access::Read, GetSize()> Flags0{ Data };
	};

	struct Reg1CChargerStatus1 : RegUtils::Register<RegOffset::ChargerStatus1, 1>
	{
		using RegUtils::Register<RegOffset::ChargerStatus1, 1>::Register;

		enum class ChgStatValues : uint8_t
		{
			NotCharging = 0,
			TrickleCharge = 1,
			PreCharge = 2,
			FastCharge = 3,
			TaperCharge = 4,
			TopOffTimerActiveCharging = 6,
			ChargindTerminationDone = 7
		};

		enum class VbusStatValues : uint8_t
		{
			NoInputOrBhotOrBcold = 0,
			UsbSdp = 1,
			UsbCdp = 2,
			UsbDcp = 3,
			HvDcp = 4,
			UnknownAdapter = 5,
			NonStandardAdapter = 6,
			Otg = 7,
			NotQualifiedAdaptor = 8,
			PoweredFromVbus = 0xB
		};

		RegUtils::Field<ChgStatValues, 5, 3, RegUtils::Access::Read, GetSize()> ChgStat{ Data };
		RegUtils::Field<VbusStatValues, 1, 4, RegUtils::Access::Read, GetSize()> VbusStat{ Data };
		RegUtils::Field<uint8_t, 0, 1, RegUtils::Access::Read, GetSize()> Bc12Done{ Data };
	};

	struct Reg1DChargerStatus2 : RegUtils::Register<RegOffset::ChargerStatus2, 1>
	{
		using RegUtils::Register<RegOffset::ChargerStatus2, 1>::Register;

		enum class IcoStatValues : uint8_t
		{
			IcoDisabled,
			IcoOptimizationInProgress,
			MaximumInputCurrent
		};

		RegUtils::Field<IcoStatValues, 6, 2, RegUtils::Access::Read, GetSize()> IcoStat{ Data };
		RegUtils::Field<uint8_t, 2, 1, RegUtils::Access::Read, GetSize()> TregStat{ Data };
		RegUtils::Field<uint8_t, 1, 1, RegUtils::Access::Read, GetSize()> DpDmStat{ Data };
		RegUtils::Field<uint8_t, 0, 1, RegUtils::Access::Read, GetSize()> VbatPresentStat{ Data };
	};

	struct Reg1EChargerStatus3 : RegUtils::Register<RegOffset::ChargerStatus3, 1>
	{
		using RegUtils::Register<RegOffset::ChargerStatus3, 1>::Register;

		enum class Flags : uint8_t
		{
			PrechgTmrExpired = (1 << 1),
			TrichgTmrExpired = (1 << 2),
			ChgTmrExpired = (1 << 3),
			VsysRegulation = (1 << 4),
			AdcDoneStat = (1 << 5),
			Acrb1Stat = (1 << 6),
			Acrb2Stat = (1 << 7)
		};

		RegUtils::Field<Flags, 0, 8, RegUtils::Access::Read, GetSize()> Flags3{ Data };
	};

	struct Reg1FChargerStatus4 : RegUtils::Register<RegOffset::ChargerStatus4, 1>
	{
		using RegUtils::Register<RegOffset::ChargerStatus4, 1>::Register;

		enum class Flags : uint8_t
		{
			TsHot = (1 << 0),
			TsWarm = (1 << 1),
			TsCool = (1 << 2),
			TsCold = (1 << 3),
			VbatOtgLow = (1 << 4)
		};

		RegUtils::Field<Flags, 0, 5, RegUtils::Access::Read, GetSize()> Flags4{ Data };
	};

	struct Reg20FaultStatus0 : RegUtils::Register<RegOffset::FaultStatus0, 1>
	{
		using RegUtils::Register<RegOffset::FaultStatus0, 1>::Register;

		enum class Flags : uint8_t
		{
			Vac1Ovp = (1 << 0),
			Vac2Ovp = (1 << 1),
			ConvOcp = (1 << 2),
			IbatOcp = (1 << 3),
			IbusOcp = (1 << 4),
			VbatOvp = (1 << 5),
			VbusOvp = (1 << 6),
			IbatReg = (1 << 7)
		};

		RegUtils::Field<Flags, 0, 8, RegUtils::Access::Read, GetSize()> Flags0{ Data };
	};

	struct Reg21FaultStatus1 : RegUtils::Register<RegOffset::FaultStatus1, 1>
	{
		using RegUtils::Register<RegOffset::FaultStatus1, 1>::Register;

		enum class Flags : uint8_t
		{
			Tshut = (1 << 2),
			OtgUvp = (1 << 4),
			OtgOvp = (1 << 5),
			VsysOvp = (1 << 6),
			VsysShort = (1 << 7)
		};

		RegUtils::Field<Flags, 0, 8, RegUtils::Access::Read, GetSize()> Flags1{ Data };
	};

	// Skipped Charget Flags

	struct Reg2CFaultMask0 : RegUtils::Register<RegOffset::FaultMask0, 1>
	{
		using RegUtils::Register<RegOffset::FaultMask0, 1>::Register;

		enum class Masks : uint8_t
		{
			Vac1Ovp = (1 << 0),
			Vac2Ovp = (1 << 1),
			ConvOcp = (1 << 2),
			IbatOcp = (1 << 3),
			IbusOcp = (1 << 4),
			VbatOvp = (1 << 5),
			VbusOvp = (1 << 6),
			IbatReg = (1 << 7)
		};

		RegUtils::Field<Masks, 0, 8, RegUtils::Access::Read, GetSize()> Masks0{ Data };
	};

	struct Reg2DFaultMask1 : RegUtils::Register<RegOffset::FaultMask1, 1>
	{
		using RegUtils::Register<RegOffset::FaultMask1, 1>::Register;

		enum class Masks : uint8_t
		{
			Tshut = (1 << 2),
			OtgUvp = (1 << 4),
			OtgOvp = (1 << 5),
			VsysOvp = (1 << 6),
			VsysShort = (1 << 7)
		};

		RegUtils::Field<Masks, 0, 8, RegUtils::Access::Read, GetSize()> Masks1{ Data };
	};

	struct Reg2EAdcControl : RegUtils::Register<RegOffset::AdcControl, 1>
	{
		using RegUtils::Register<RegOffset::AdcControl, 1>::Register;

		RegUtils::Field<uint8_t, 7, 1, RegUtils::Access::ReadWrite, GetSize()> AdcEn{ Data };
		RegUtils::Field<uint8_t, 6, 1, RegUtils::Access::ReadWrite, GetSize()> AdcOneShot{ Data };

		/// <summary>
		/// <para>0 - 15 bit effective resolution</para>
		/// <para>1 - 14 bit effective resolution</para>
		/// <para>2 - 13 bit effective resolution</para>
		/// <para>3 - 12 bit effective resolution</para>
		/// </summary>
		RegUtils::Field<uint8_t, 4, 2, RegUtils::Access::ReadWrite, GetSize()> AdcSampleSpeed{ Data };
		RegUtils::Field<uint8_t, 3, 1, RegUtils::Access::ReadWrite, GetSize()> AdcAvg{ Data };
		RegUtils::Field<uint8_t, 2, 1, RegUtils::Access::ReadWrite, GetSize()> AdcAvgInit{ Data };
	};

	// Skipped ADC disable registers

	struct Reg31IbusAdc : RegUtils::Register<RegOffset::IbusAdc, 2>
	{
		using RegUtils::Register<RegOffset::IbusAdc, 2>::Register;

		RegUtils::Field<uint16_t, 0, 16, RegUtils::Access::Read, GetSize()> IbusAdc{ Data };

		constexpr MilliAmperes GetIbusCurrent() const
		{
			uint16_t value = IbusAdc.Get();
			int16_t sValue = RegUtils::ConvertTwosComplement(value);
			return MilliAmperes(sValue);
		}
	};

	struct Reg33IbatAdc : RegUtils::Register<RegOffset::IbatAdc, 2>
	{
		using RegUtils::Register<RegOffset::IbatAdc, 2>::Register;

		RegUtils::Field<uint16_t, 0, 16, RegUtils::Access::Read, GetSize()> IbatAdc{ Data };

		constexpr MilliAmperes GetIbatCurrent() const
		{
			uint16_t value = IbatAdc.Get();
			int16_t sValue = RegUtils::ConvertTwosComplement(value);
			return MilliAmperes(sValue);
		}
	};

	struct Reg35VbusAdc : RegUtils::Register<RegOffset::VbusAdc, 2>
	{
		using RegUtils::Register<RegOffset::VbusAdc, 2>::Register;

		RegUtils::Field<uint16_t, 0, 16, RegUtils::Access::Read, GetSize()> VbusAdc{ Data };

		constexpr MilliVolts GetVbusVoltage() const
		{
			uint16_t value = VbusAdc.Get();
			return MilliVolts(value);
		}
	};

	struct Reg3BVbatAdc : RegUtils::Register<RegOffset::VbatAdc, 2>
	{
		using RegUtils::Register<RegOffset::VbatAdc, 2>::Register;

		RegUtils::Field<uint16_t, 0, 16, RegUtils::Access::Read, GetSize()> VbatAdc{ Data };

		constexpr MilliVolts GetVbatVoltage() const
		{
			uint16_t value = VbatAdc.Get();
			return MilliVolts(value);
		}
	};

	struct Reg3DVsysAdc : RegUtils::Register<RegOffset::VsysAdc, 2>
	{
		using RegUtils::Register<RegOffset::VsysAdc, 2>::Register;

		RegUtils::Field<uint16_t, 0, 16, RegUtils::Access::Read, GetSize()> VsysAdc{ Data };

		constexpr MilliVolts GetVsysVoltage() const
		{
			uint16_t value = VsysAdc.Get();
			return MilliVolts(value);
		}
	};

	struct Reg3FTsAdc : RegUtils::Register<RegOffset::TsAdc, 2>
	{
		using RegUtils::Register<RegOffset::TsAdc, 2>::Register;

		RegUtils::Field<uint16_t, 0, 16, RegUtils::Access::Read, GetSize()> TsAdc{ Data };
	};

	struct Reg41TdieAdc : RegUtils::Register<RegOffset::TdieAdc, 2>
	{
		using RegUtils::Register<RegOffset::TdieAdc, 2>::Register;

		/// <summary>
		/// Die Temperature. Range: [-40, 150] Celcius. Bit step size: 0.5C. Reported in 2's complement,
		/// </summary>
		RegUtils::Field<uint16_t, 0, 16, RegUtils::Access::Read, GetSize()> TdieAdc{ Data };
	};

	struct Reg43DpAdc : RegUtils::Register<RegOffset::DpAdc, 2>
	{
		using RegUtils::Register<RegOffset::DpAdc, 2>::Register;

		RegUtils::Field<uint16_t, 0, 16, RegUtils::Access::Read, GetSize()> DpAdc{ Data };

		constexpr MilliVolts GetVbusVoltage() const
		{
			uint16_t value = DpAdc.Get();
			return MilliVolts(value);
		}
	};

	struct Reg45DmAdc : RegUtils::Register<RegOffset::DmAdc, 2>
	{
		using RegUtils::Register<RegOffset::DmAdc, 2>::Register;

		RegUtils::Field<uint16_t, 0, 16, RegUtils::Access::Read, GetSize()> DmAdc{ Data };

		constexpr MilliVolts GetVbusVoltage() const
		{
			uint16_t value = DmAdc.Get();
			return MilliVolts(value);
		}
	};
}