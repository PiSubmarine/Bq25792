#pragma once

#include "PiSubmarine/RegUtils.h"
#include "PiSubmarine/Bq25792/Units.h"
#include "PiSubmarine/Api/Internal/I2C/DriverConcept.h"
#include <functional>
#include <bitset>

namespace PiSubmarine::Bq25792
{
	enum class RegOffset : uint8_t
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

	/// <summary>
	/// Bit position at register offset determines register size. 0 - 1 byte, 1 - 2 bytes. For registers below 0x40.
	/// </summary>
	constexpr uint64_t RegSizesA =
		(1ULL << static_cast<uint8_t>(RegOffset::ChargeVoltageLimit)) |
		(1ULL << static_cast<uint8_t>(RegOffset::ChargeCurrentLimit)) |
		(1ULL << static_cast<uint8_t>(RegOffset::InputCurrentLimit)) |
		(1ULL << static_cast<uint8_t>(RegOffset::VotgRegulation)) |
		(1ULL << static_cast<uint8_t>(RegOffset::IcoCurrentLimit)) |
		(1ULL << static_cast<uint8_t>(RegOffset::IbusAdc)) |
		(1ULL << static_cast<uint8_t>(RegOffset::IbatAdc)) |
		(1ULL << static_cast<uint8_t>(RegOffset::VbusAdc)) |
		(1ULL << static_cast<uint8_t>(RegOffset::Vac1Adc)) |
		(1ULL << static_cast<uint8_t>(RegOffset::Vac2Adc)) |
		(1ULL << static_cast<uint8_t>(RegOffset::VsysAdc)) |
		(1ULL << static_cast<uint8_t>(RegOffset::TsAdc));


	/// <summary>
	/// Bit position at register offset determines register size. 0 - 1 byte, 1 - 2 bytes. For registers above 0x40. Shifted by 64 positions.
	/// </summary>
	constexpr uint8_t RegSizesB =
		(1ULL << (static_cast<uint8_t>(RegOffset::TdieAdc) - 64)) |
		(1ULL << (static_cast<uint8_t>(RegOffset::DpAdc) - 64)) |
		(1ULL << (static_cast<uint8_t>(RegOffset::DmAdc) - 64));

	constexpr uint8_t GetRegisterSize(RegOffset reg)
	{
		uint8_t bitPos = static_cast<uint8_t>(reg);
		if (bitPos < 64)
		{
			return RegSizesA & (1ULL << bitPos) ? 2 : 1;
		}
		else
		{
			return RegSizesB & (1ULL << (bitPos - 64)) ? 2 : 1;
		}
	}

	enum class ChargerStatus0Flags : uint8_t
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

	enum class ChargeStatus : uint8_t
	{
		NotCharging = 0,
		TrickleCharge = 1,
		PreCharge = 2,
		FastCharge = 3,
		TaperCharge = 4,
		TopOffTimerActiveCharging = 6,
		ChargindTerminationDone = 7
	};

	enum class VbusStatus : uint8_t
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

	enum class IcoStatus : uint8_t
	{
		IcoDisabled,
		IcoOptimizationInProgress,
		MaximumInputCurrent
	};

	enum class IbatReg
	{
		Amps3 = 0,
		Amps4 = 1,
		Amps5 = 2,
		Disable = 3
	};

	template<PiSubmarine::Api::Internal::I2C::DriverConcept I2CDriver>
	class Device
	{
	public:
		constexpr static uint8_t Address = 0x6B;

		Device(I2CDriver& driver) : m_Driver(driver)
		{

		}

		/// <summary>
		/// Returns true if there is a pending read/write I2C transation.
		/// </summary>
		/// <returns>True if transaction not finished.</returns>
		bool IsTransactionInProgress()
		{
			return m_IsTransactionInProgress;
		}

		/// <summary>
		/// Returns true if previous transaction failed. Cleared to false upon new Read or Write.
		/// </summary>
		/// <returns>True if has error.</returns>
		bool HasError()
		{
			return m_HasError;
		}

		/// <summary>
		/// Reads all registers.
		/// </summary>
		/// <returns>True if transaction was successfully started.</returns>
		bool Read()
		{
			std::bitset<MemorySize> regs;
			regs.set();
			return Read(0, m_ChargerMemoryBuffer.data(), m_ChargerMemoryBuffer.size(), regs);
		}

		/// <summary>
		/// Reads specific register.
		/// </summary>
		/// <param name="reg">Register offset</param>
		/// <returns>True if transaction was successfully started.</returns>
		bool Read(RegOffset reg)
		{
			std::bitset<MemorySize> regs;
			regs.set(RegUtils::ToInt(reg));
			size_t regSize = GetRegisterSize(reg);
			return Read(static_cast<uint8_t>(reg), m_ChargerMemoryBuffer.data() + static_cast<size_t>(reg), regSize, regs);
		}

		/// <summary>
		/// Writes all registers.
		/// </summary>
		/// <returns>True if transaction was successfully started.</returns>
		bool Write()
		{
			std::bitset<MemorySize> regs;
			regs.set();
			return Write(0, m_ChargerMemoryBuffer.data(), m_ChargerMemoryBuffer.size(), regs);
		}

		/// <summary>
		/// Writes specific register.
		/// </summary>
		/// <param name="reg">Register offset</param>
		/// <returns>True if transaction was successfully started.</returns>
		bool Write(RegOffset reg)
		{
			size_t regSize = GetRegisterSize(reg);
			std::bitset<MemorySize> regs;
			regs.set(RegUtils::ToInt(reg));
			return Write(static_cast<uint8_t>(reg), m_ChargerMemoryBuffer.data() + static_cast<size_t>(reg), regSize, regs);
		}

		/// <summary>
		/// Writes all dirty registers in a sequence of transactions.
		/// </summary>
		/// <returns>True if transaction was successfully started. False if there was an error or no register was dirty.</returns>
		bool WriteDirty()
		{
			if (m_IsTransactionInProgress)
			{
				return false;
			}

			m_HasError = false;
			m_IsTransactionInProgress = true;

			return WriteDirtyInternal(RegOffset{0});
		}

		bool HasDirtyRegisters()
		{
			return m_DirtyRegs.any();
		}

		/// <summary>
		/// Gets Minimal System Voltage (VSYSMIN) from Memory Buffer.
		/// </summary>
		/// <returns>VSYSMIN in mV.</returns>
		MilliVolts GetMinimalSystemVoltage() const
		{
			uint8_t vsysMin = RegUtils::Read<uint8_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::MinimalSystemVoltage), 0, 6);
			return 2500_mV + MilliVolts(vsysMin) * 250_mV;
		}

		/// <summary>
		/// Sets minimal system voltage. Range: 2500mV - 16000mV, bit step size: 250mV
		/// </summary>
		/// <param name="valueMv">Voltage in mV</param>
		void SetMinimalSystemVoltage(MilliVolts valueMv)
		{
			uint8_t value = (valueMv.Value - 2500) / 250;
			RegUtils::Write<uint8_t, std::endian::big>(value, m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::MinimalSystemVoltage), 0, 6);
			m_DirtyRegs[RegUtils::ToInt(RegOffset::MinimalSystemVoltage)] = true;
		}

		/// <summary>
		/// Gets maxium charge current.
		/// </summary>
		/// <returns>Current in mA</returns>
		constexpr MilliAmperes GetChargeCurrentLimit() const
		{
			uint16_t Ichg = RegUtils::Read<uint16_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargeCurrentLimit), 0, 9);
			return MilliAmperes(Ichg) * 10_mA;
		}

		/// <summary>
		/// Sets maximum charge current. Range: 50mA - 5000mA, bit step size: 10mA.
		/// </summary>
		/// <param name="valueMa">Current in mA</param>
		void SetChargeCurrentLimit(MilliAmperes valueMa)
		{
			uint16_t value = valueMa.Value / 10;
			RegUtils::Write<uint16_t, std::endian::big>(value, m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargeCurrentLimit), 0, 9);
			m_DirtyRegs[RegUtils::ToInt(RegOffset::ChargeCurrentLimit)] = true;
		}

		/*
		* struct Reg14ChargerControl5 : RegUtils::Register<RegOffset::ChargerControl5, 1>
		{
			using RegUtils::Register<RegOffset::ChargerControl5, 1>::Register;

			enum class IbatRegValues
			{
				Amps3 = 0,
				Amps4 = 1,
				Amps5 = 2,
				Disable = 3
			};

			RegUtils::Field<uint8_t, 7, 1, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> SfetPresent{ Data };
			RegUtils::Field<uint8_t, 5, 1, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> EnIbat{ Data };
			RegUtils::Field<IbatRegValues, 3, 2, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> IbatReg{ Data };
			RegUtils::Field<uint8_t, 2, 1, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> EnIinDpm{ Data };
			RegUtils::Field<uint8_t, 1, 1, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> EnExtLim{ Data };
			RegUtils::Field<uint8_t, 0, 1, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> EnBatoc{ Data };
		};
		*/

		IbatReg GetOtgMaxCurrent() const
		{
			return RegUtils::Read<IbatReg, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerControl5), 3, 2);
		}

		void SetOtgMaxCurrent(IbatReg value)
		{
			RegUtils::Write<IbatReg, std::endian::big>(value, m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerControl5), 0, 1);
			m_DirtyRegs[RegUtils::ToInt(RegOffset::ChargerControl5)] = true;
		}

		bool IsSfetPresent() const
		{
			return RegUtils::Read<uint8_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerControl5), 7, 1);
		}

		void SetSfetPresent(bool value)
		{
			RegUtils::Write<IbatReg, std::endian::big>(value, m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerControl5), 7, 1);
			m_DirtyRegs[RegUtils::ToInt(RegOffset::ChargerControl5)] = true;
		}

		bool IsDischargeCurrentSensingEnabled() const
		{
			return RegUtils::Read<uint8_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerControl5), 5, 1);
		}

		void SetDischargeCurrentSensingEnabled(bool value)
		{
			RegUtils::Write<uint8_t, std::endian::big>(value, m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerControl5), 5, 1);
			m_DirtyRegs[RegUtils::ToInt(RegOffset::ChargerControl5)] = true;
		}

		bool IsIlimHizCurrentLimitEnabled() const
		{
			return RegUtils::Read<uint8_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerControl5), 1, 1);
		}

		void SetIlimHizCurrentLimitEnabled(bool value)
		{
			RegUtils::Write<uint8_t, std::endian::big>(value, m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerControl5), 1, 1);
			m_DirtyRegs[RegUtils::ToInt(RegOffset::ChargerControl5)] = true;
		}

		bool IsDischargeOcpEnabled() const
		{
			return RegUtils::Read<uint8_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerControl5), 0, 1);
		}

		void SetDischargeOcpEnabled(bool value)
		{
			RegUtils::Write<uint8_t, std::endian::big>(value, m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerControl5), 0, 1);
			m_DirtyRegs[RegUtils::ToInt(RegOffset::ChargerControl5)] = true;
		}

		void SetTsIgnore(bool value)
		{
			RegUtils::Write<uint8_t, std::endian::big>(value, m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::NtcControl1), 0, 1);
			m_DirtyRegs[RegUtils::ToInt(RegOffset::NtcControl1)] = true;
		}

		bool GetTsIgnore()
		{
			return RegUtils::Read<uint8_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::NtcControl1), 0, 1);
		}

		void SetWdRst(bool value)
		{
			RegUtils::Write<uint8_t, std::endian::big>(value, m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerControl1), 3, 1);
			m_DirtyRegs[RegUtils::ToInt(RegOffset::ChargerControl1)] = true;
		}

		bool GetWdRst()
		{
			return RegUtils::Read<uint8_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerControl1), 3, 1);
		}
		
		/// <summary>
		/// <para>0 - 15 bit effective resolution</para>
		/// <para>1 - 14 bit effective resolution</para>
		/// <para>2 - 13 bit effective resolution</para>
		/// <para>3 - 12 bit effective resolution</para>
		/// </summary>
		/// <returns></returns>
		uint8_t GetAdcSampleSpeed() const
		{
			return RegUtils::Read<uint8_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::AdcControl), 4, 2);
		}

		/// <summary>
		/// <para>0 - 15 bit effective resolution</para>
		/// <para>1 - 14 bit effective resolution</para>
		/// <para>2 - 13 bit effective resolution</para>
		/// <para>3 - 12 bit effective resolution</para>
		/// </summary>
		/// <param name="value">Sample speed in range [0, 3]</param>
		void SetAdcSampleSpeed(uint8_t value)
		{
			RegUtils::Write<uint8_t, std::endian::big>(value, m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::AdcControl), 4, 2);
			m_DirtyRegs[RegUtils::ToInt(RegOffset::ChargerControl1)] = true;
		}

		bool IsAdcEnabled() const
		{
			return RegUtils::Read<uint8_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::AdcControl), 7, 1);
		}

		void SetAdcEnabled(bool value)
		{
			RegUtils::Write<uint8_t, std::endian::big>(value, m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::AdcControl), 7, 1);
			m_DirtyRegs[RegUtils::ToInt(RegOffset::AdcControl)] = true;
		}

		ChargerStatus0Flags GetChargerStatus0() const
		{
			return RegUtils::Read<ChargerStatus0Flags, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerStatus0), 0, 8);
		}

		ChargeStatus GetChargeStatus() const
		{
			return RegUtils::Read<ChargeStatus, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerStatus1), 5, 3);
		}

		VbusStatus GetVbusStatus() const
		{
			return RegUtils::Read<VbusStatus, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerStatus1), 1, 4);
		}

		bool IsBc12DetectionComplete() const
		{
			return RegUtils::Read<uint8_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerStatus1), 0, 1);
		}

		IcoStatus GetIcoStatus() const
		{
			return RegUtils::Read<IcoStatus, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerStatus2), 6, 2);
		}

		bool IsInThermalRegulation() const
		{
			return RegUtils::Read<uint8_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerStatus2), 2, 1);
		}

		bool IsDpDmDetectionOngoing() const
		{
			return RegUtils::Read<uint8_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerStatus2), 1, 1);
		}

		bool IsBatteryPresent() const
		{
			return RegUtils::Read<uint8_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerStatus2), 0, 1);
		}

		MilliAmperes GetIbusCurrent() const
		{
			auto isubAdc = RegUtils::Read<uint16_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::IbusAdc), 0, 16);
			int16_t sValue = RegUtils::ConvertTwosComplement(isubAdc);
			return MilliAmperes(sValue);
		}

		MilliAmperes GetIbatCurrent() const
		{
			uint16_t value = RegUtils::Read<uint16_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::IbatAdc), 0, 16);
			int16_t sValue = RegUtils::ConvertTwosComplement(value);
			return MilliAmperes(sValue);
		}

		MilliVolts GetVbusVoltage() const
		{
			uint16_t value = RegUtils::Read<uint16_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::VbusAdc), 0, 16);
			return MilliVolts(value);
		}

		MilliVolts GetVbatVoltage() const
		{
			uint16_t value = RegUtils::Read<uint16_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::VbatAdc), 0, 16);
			return MilliVolts(value);
		}

		MilliVolts GetVsysVoltage() const
		{
			uint16_t value = RegUtils::Read<uint16_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::VsysAdc), 0, 16);
			return MilliVolts(value);
		}

		uint16_t GetTsPercentage() const
		{
			uint16_t value = RegUtils::Read<uint16_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::TsAdc), 0, 16);
			return value;
		}

		Celcius GetDieTemperature() const
		{
			uint16_t value = RegUtils::Read<uint16_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::TdieAdc), 0, 16);
			int16_t sValue = RegUtils::ConvertTwosComplement(value);
			return Celcius(sValue);
		}

		MilliVolts GetUsbDataPlusVoltage() const
		{
			uint16_t value = RegUtils::Read<uint16_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::DpAdc), 0, 16);
			return MilliVolts(value);
		}

		MilliVolts GetUsbDataMinusVoltage() const
		{
			uint16_t value = RegUtils::Read<uint16_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::DmAdc), 0, 16);
			return MilliVolts(value);
		}

	private:
		constexpr static size_t MemorySize = 0x49;

		I2CDriver& m_Driver;
		std::array<uint8_t, MemorySize> m_ChargerMemoryBuffer{0};
		bool m_IsTransactionInProgress = false;
		bool m_HasError = false;
		std::bitset<MemorySize> m_DirtyRegs{ 0 };

		bool Read(uint8_t offset, uint8_t* data, size_t size, const std::bitset<MemorySize>& regs)
		{
			if (m_IsTransactionInProgress)
			{
				return false;
			}

			m_HasError = !m_Driver.Write(Address, &offset, 1);
			if (m_HasError)
			{
				return false;
			}

			bool transactionStarted = m_Driver.ReadAsync(Address, data, size, [this, regs](uint8_t cbAddress, bool cbOk) {ReadCallback(cbAddress, regs, cbOk); });
			if (transactionStarted)
			{
				m_IsTransactionInProgress = true;
			}

			return transactionStarted;
		}

		bool Write(uint8_t offset, uint8_t* data, size_t size, const std::bitset<MemorySize>& regs)
		{
			if (m_IsTransactionInProgress)
			{
				return false;
			}

			m_HasError = false;

			std::vector<uint8_t> buffer;
			buffer.resize(size + 1);
			buffer[0] = offset;
			memcpy(buffer.data() + 1, data, size);

			bool transactionStarted = m_Driver.WriteAsync(Address, buffer.data(), buffer.size(), [this, regs](uint8_t cbAddress, bool cbOk) {WriteCallback(cbAddress, regs, cbOk); });
			if (transactionStarted)
			{
				m_IsTransactionInProgress = true;
			}

			return transactionStarted;
		}

		void ReadCallback(uint8_t deviceAddress, std::bitset<MemorySize> regs, bool ok)
		{
			m_HasError = !ok;
			m_IsTransactionInProgress = false;

			if (ok)
			{
				m_DirtyRegs &= ~regs;
			}
		}

		void WriteCallback(uint8_t deviceAddress, std::bitset<MemorySize> regs, bool ok)
		{
			m_HasError = !ok;
			m_IsTransactionInProgress = false;

			if (ok)
			{
				m_DirtyRegs &= ~regs;
			}
		}

		bool WriteDirtyInternal(RegOffset regNext)
		{
			for (size_t i = RegUtils::ToInt(regNext); i < m_DirtyRegs.size(); i++)
			{
				if (!m_DirtyRegs[i])
				{
					continue;
				}
				RegOffset reg = static_cast<RegOffset>(i);
				
				uint8_t regSize = GetRegisterSize(reg);
				std::vector<uint8_t> buffer;
				buffer.resize(regSize + 1);
				buffer[0] = i;
				memcpy(buffer.data() + 1, m_ChargerMemoryBuffer.data() + i, regSize);
				return m_Driver.WriteAsync(Address, buffer.data(), buffer.size(), [this, reg](uint8_t cbAddress, bool cbOk) {WriteDirtyCallback(cbAddress, reg, cbOk); });
			}
			return false;
		}

		void WriteDirtyCallback(uint8_t deviceAddress, RegOffset reg, bool ok)
		{
			if (!ok)
			{
				m_HasError = true;
				m_IsTransactionInProgress = false;
				return;
			}

			m_HasError = false;
			m_DirtyRegs[RegUtils::ToInt(reg)] = false;
			if (m_DirtyRegs == 0)
			{
				m_IsTransactionInProgress = false;
				return;
			}

			if (!WriteDirtyInternal(static_cast<RegOffset>(RegUtils::ToInt(reg) + 1)))
			{
				m_HasError = true;
				m_IsTransactionInProgress = false;
				return;
			}
		}
	};

	/*

	struct Reg00MinimalSystemVoltage : RegUtils::Register<RegOffset::MinimalSystemVoltage, 1>
	{
		using RegUtils::Register<RegOffset::MinimalSystemVoltage, 1>::Register;

		RegUtils::Field<uint8_t, 0, 6, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> VsysMin{ Data };

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

		RegUtils::Field<uint16_t, 0, 11, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> Vreg{ Data };

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

		RegUtils::Field<uint16_t, 0, 9, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> Ichg{ Data };

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

		RegUtils::Field<uint8_t, 0, 8, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> Vindpm{ Data };

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

		RegUtils::Field<uint16_t, 0, 9, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> Iindpm{ Data };

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

		RegUtils::Field<PrechargeVoltageThreholds, 6, 2, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> VbatLowv{ Data };
		RegUtils::Field<uint8_t, 0, 6, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> Iprechg{ Data };
		
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

		RegUtils::Field<uint8_t, 6, 1, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> RegRst{ Data };
		RegUtils::Field<uint8_t, 0, 5, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> Iterm{ Data };

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

		RegUtils::Field<uint8_t, 6, 2, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> Cell{ Data };
		RegUtils::Field<BatteryRechargeDeglichTimes, 4, 2, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> Trechg{ Data };
		RegUtils::Field<uint8_t, 0, 4, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> Vrechg{ Data };
		
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
			Vrechg.Set(value);
		}
	};

	struct Reg0BVotgRegulation : RegUtils::Register<RegOffset::VotgRegulation, 2>
	{
		using RegUtils::Register<RegOffset::VotgRegulation, 2>::Register;

		RegUtils::Field<uint16_t, 0, 11, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> Votg{ Data };

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

		RegUtils::Field<PrechargeSafetyTimerValues, 7, 1, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> PrechgTmr{ Data };
		RegUtils::Field<uint8_t, 0, 7, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> Iotg{ Data };

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

		RegUtils::Field<TopOffTimerValues, 6, 2, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> TopOffTimer{ Data };
		RegUtils::Field<uint8_t, 5, 1, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> EnTrichgTmr{ Data };
		RegUtils::Field<uint8_t, 4, 1, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> EnPrechgTmr{ Data };
		RegUtils::Field<uint8_t, 3, 1, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> EnChgTmr{ Data };
		RegUtils::Field<FastChargeTimerValues, 1, 2, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> ChgTmr{ Data };
		RegUtils::Field<uint8_t, 0, 1, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> Tmr2xEn{ Data };
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

		RegUtils::Field<Flags, 0, 7, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> Flags0{ Data };
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

		RegUtils::Field<VacOvpThresholds, 4, 2, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> VacOvp{ Data };

		// Writing 1 resets Watchdog
		RegUtils::Field<uint8_t, 3, 1, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> WdRst{ Data };

		RegUtils::Field<WatchdogValues, 0, 3, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> Watchdog{ Data };
	};

	struct Reg11ChargerControl2 : RegUtils::Register<RegOffset::ChargerControl2, 1>
	{
		using RegUtils::Register<RegOffset::ChargerControl2, 1>::Register;

		enum class SdrvControlValues
		{
			Idle = 0,
			ShutdownMode,
			ShipMode,
			SystemPowerReset
		};
		
		RegUtils::Field<uint8_t, 7, 1, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> ForceIndet{ Data };
		RegUtils::Field<uint8_t, 6, 1, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> AutoIndetEn{ Data };
		RegUtils::Field<uint8_t, 5, 1, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> En12V{ Data };
		RegUtils::Field<uint8_t, 4, 1, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> En9V{ Data };
		RegUtils::Field<uint8_t, 3, 1, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> HvDcp{ Data };
		RegUtils::Field<SdrvControlValues, 1, 2, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> SdrvCtrl{ Data };
		RegUtils::Field<uint8_t, 0, 1, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> SdrvDly{ Data };
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

		RegUtils::Field<Flags, 0, 8, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> Flags3{ Data };
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

		RegUtils::Field<Flags, 0, 8, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> Flags4{ Data };
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

		RegUtils::Field<uint8_t, 7, 1, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> SfetPresent{ Data };
		RegUtils::Field<uint8_t, 5, 1, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> EnIbat{ Data };
		RegUtils::Field<IbatRegValues, 3, 2, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> IbatReg{ Data };
		RegUtils::Field<uint8_t, 2, 1, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> EnIinDpm{ Data };
		RegUtils::Field<uint8_t, 1, 1, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> EnExtLim{ Data };
		RegUtils::Field<uint8_t, 0, 1, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> EnBatoc{ Data };
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

		RegUtils::Field<ThermalRegulationThresholds, 6, 2, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> Treg{ Data };
		RegUtils::Field<ThermalShutdownThresholds, 4, 2, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> Tshut{ Data };
		RegUtils::Field<uint8_t, 3, 1, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> VbusPdEn{ Data };
		RegUtils::Field<uint8_t, 2, 1, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> Vac1PdEn{ Data };
		RegUtils::Field<uint8_t, 1, 1, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> Vac2PdEn{ Data };
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


		RegUtils::Field<JeitaVset, 5, 3, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> JeitaVset{ Data };
		RegUtils::Field<JeitaIset, 3, 2, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> JeitaSetHot{ Data };
		RegUtils::Field<JeitaIset, 1, 2, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> JeitaSetCold{ Data };
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

		RegUtils::Field<TsCoolValues, 6, 2, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> TsCool{ Data };
		RegUtils::Field<TsWarmValues, 4, 2, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> TsWarm{ Data };
		RegUtils::Field<BHotValues, 2, 2, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> BHot{ Data };
		RegUtils::Field<BColdValues, 1, 1, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> BCold{ Data };
		RegUtils::Field<uint8_t, 0, 1, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> TsIgnore{ Data };
	};

	struct Reg19IcoCurrentLimit : RegUtils::Register<RegOffset::IcoCurrentLimit, 2>
	{
		using RegUtils::Register<RegOffset::IcoCurrentLimit, 2>::Register;

		RegUtils::Field<uint16_t, 0, 9, RegUtils::Access::Read, GetSize(), std::endian::big> IcoIlim{ Data };

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

		RegUtils::Field<Flags, 0, 8, RegUtils::Access::Read, GetSize(), std::endian::big> Flags0{ Data };
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

		RegUtils::Field<ChgStatValues, 5, 3, RegUtils::Access::Read, GetSize(), std::endian::big> ChgStat{ Data };
		RegUtils::Field<VbusStatValues, 1, 4, RegUtils::Access::Read, GetSize(), std::endian::big> VbusStat{ Data };
		RegUtils::Field<uint8_t, 0, 1, RegUtils::Access::Read, GetSize(), std::endian::big> Bc12Done{ Data };
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

		RegUtils::Field<IcoStatValues, 6, 2, RegUtils::Access::Read, GetSize(), std::endian::big> IcoStat{ Data };
		RegUtils::Field<uint8_t, 2, 1, RegUtils::Access::Read, GetSize(), std::endian::big> TregStat{ Data };
		RegUtils::Field<uint8_t, 1, 1, RegUtils::Access::Read, GetSize(), std::endian::big> DpDmStat{ Data };
		RegUtils::Field<uint8_t, 0, 1, RegUtils::Access::Read, GetSize(), std::endian::big> VbatPresentStat{ Data };
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

		RegUtils::Field<Flags, 0, 8, RegUtils::Access::Read, GetSize(), std::endian::big> Flags3{ Data };
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

		RegUtils::Field<Flags, 0, 5, RegUtils::Access::Read, GetSize(), std::endian::big> Flags4{ Data };
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

		RegUtils::Field<Flags, 0, 8, RegUtils::Access::Read, GetSize(), std::endian::big> Flags0{ Data };
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

		RegUtils::Field<Flags, 0, 8, RegUtils::Access::Read, GetSize(), std::endian::big> Flags1{ Data };
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

		RegUtils::Field<Masks, 0, 8, RegUtils::Access::Read, GetSize(), std::endian::big> Masks0{ Data };
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

		RegUtils::Field<Masks, 0, 8, RegUtils::Access::Read, GetSize(), std::endian::big> Masks1{ Data };
	};

	struct Reg2EAdcControl : RegUtils::Register<RegOffset::AdcControl, 1>
	{
		using RegUtils::Register<RegOffset::AdcControl, 1>::Register;

		RegUtils::Field<uint8_t, 7, 1, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> AdcEn{ Data };
		RegUtils::Field<uint8_t, 6, 1, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> AdcOneShot{ Data };

		/// <summary>
		/// <para>0 - 15 bit effective resolution</para>
		/// <para>1 - 14 bit effective resolution</para>
		/// <para>2 - 13 bit effective resolution</para>
		/// <para>3 - 12 bit effective resolution</para>
		/// </summary>
		RegUtils::Field<uint8_t, 4, 2, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> AdcSampleSpeed{ Data };
		RegUtils::Field<uint8_t, 3, 1, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> AdcAvg{ Data };
		RegUtils::Field<uint8_t, 2, 1, RegUtils::Access::ReadWrite, GetSize(), std::endian::big> AdcAvgInit{ Data };
	};

	// Skipped ADC disable registers

	struct Reg31IbusAdc : RegUtils::Register<RegOffset::IbusAdc, 2>
	{
		using RegUtils::Register<RegOffset::IbusAdc, 2>::Register;

		RegUtils::Field<uint16_t, 0, 16, RegUtils::Access::Read, GetSize(), std::endian::big> IbusAdc{ Data };

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

		RegUtils::Field<uint16_t, 0, 16, RegUtils::Access::Read, GetSize(), std::endian::big> IbatAdc{ Data };

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

		RegUtils::Field<uint16_t, 0, 16, RegUtils::Access::Read, GetSize(), std::endian::big> VbusAdc{ Data };

		constexpr MilliVolts GetVbusVoltage() const
		{
			uint16_t value = VbusAdc.Get();
			return MilliVolts(value);
		}
	};

	struct Reg3BVbatAdc : RegUtils::Register<RegOffset::VbatAdc, 2>
	{
		using RegUtils::Register<RegOffset::VbatAdc, 2>::Register;

		RegUtils::Field<uint16_t, 0, 16, RegUtils::Access::Read, GetSize(), std::endian::big> VbatAdc{ Data };

		constexpr MilliVolts GetVbatVoltage() const
		{
			uint16_t value = VbatAdc.Get();
			return MilliVolts(value);
		}
	};

	struct Reg3DVsysAdc : RegUtils::Register<RegOffset::VsysAdc, 2>
	{
		using RegUtils::Register<RegOffset::VsysAdc, 2>::Register;

		RegUtils::Field<uint16_t, 0, 16, RegUtils::Access::Read, GetSize(), std::endian::big> VsysAdc{ Data };

		constexpr MilliVolts GetVsysVoltage() const
		{
			uint16_t value = VsysAdc.Get();
			return MilliVolts(value);
		}
	};

	struct Reg3FTsAdc : RegUtils::Register<RegOffset::TsAdc, 2>
	{
		using RegUtils::Register<RegOffset::TsAdc, 2>::Register;

		RegUtils::Field<uint16_t, 0, 16, RegUtils::Access::Read, GetSize(), std::endian::big> TsAdc{ Data };
	};

	struct Reg41TdieAdc : RegUtils::Register<RegOffset::TdieAdc, 2>
	{
		using RegUtils::Register<RegOffset::TdieAdc, 2>::Register;

		/// <summary>
		/// Die Temperature. Range: [-40, 150] Celcius. Bit step size: 0.5C. Reported in 2's complement,
		/// </summary>
		RegUtils::Field<uint16_t, 0, 16, RegUtils::Access::Read, GetSize(), std::endian::big> TdieAdc{ Data };
	};

	struct Reg43DpAdc : RegUtils::Register<RegOffset::DpAdc, 2>
	{
		using RegUtils::Register<RegOffset::DpAdc, 2>::Register;

		RegUtils::Field<uint16_t, 0, 16, RegUtils::Access::Read, GetSize(), std::endian::big> DpAdc{ Data };

		constexpr MilliVolts GetVbusVoltage() const
		{
			uint16_t value = DpAdc.Get();
			return MilliVolts(value);
		}
	};

	struct Reg45DmAdc : RegUtils::Register<RegOffset::DmAdc, 2>
	{
		using RegUtils::Register<RegOffset::DmAdc, 2>::Register;

		RegUtils::Field<uint16_t, 0, 16, RegUtils::Access::Read, GetSize(), std::endian::big> DmAdc{ Data };

		constexpr MilliVolts GetVbusVoltage() const
		{
			uint16_t value = DmAdc.Get();
			return MilliVolts(value);
		}
	};

	*/
}