#include <span>
#include <vector>
#include <cstring> // needed for memcpy on STM32
#include "PiSubmarine/Bq25792/Device.h"

namespace PiSubmarine::Bq25792
{
	Device::Device(PiSubmarine::I2C::Api::IDriver& driver) : m_Driver(driver)
	{
	}

	Error::Api::Result<MilliVolts> Device::GetMinimalSystemVoltage() const
	{
		auto field = ReadField<RegOffset::MinimalSystemVoltage>(0, 6);
		if (field.has_value())
		{
			return 2500_mV + MilliVolts(field.value()) * 250_mV;
		}
		return std::unexpected(field.error());
	}

	Error::Api::Result<void> Device::SetMinimalSystemVoltage(MilliVolts valueMv) const
	{
		uint8_t value = (valueMv.Value - 2500) / 250;
		return WriteField<RegOffset::MinimalSystemVoltage>(value, 0, 6);
	}

	Error::Api::Result<MilliVolts> Device::GetChargeVoltageLimit() const
	{
		auto field = ReadField<RegOffset::ChargeVoltageLimit>(0, 11);
		if (field.has_value())
		{
			return MilliVolts(field.value()) * 10_mV;
		}
		return std::unexpected(field.error());
	}

	Error::Api::Result<void> Device::SetChargeVoltageLimit(MilliVolts valueMv) const
	{
		uint16_t value = valueMv.Value / 10;
		return WriteField<RegOffset::ChargeVoltageLimit>(value, 0, 11);
	}

	Error::Api::Result<MilliAmperes> Device::GetChargeCurrentLimit() const
	{
		auto current = ReadField<RegOffset::ChargeCurrentLimit>(0, 9);
		if (!current.has_value())
		{
			return std::unexpected(current.error());
		}
		return MilliAmperes(current.value()) * 10_mA;
	}


	Error::Api::Result<void> Device::SetChargeCurrentLimit(MilliAmperes valueMa) const
	{
		uint16_t value = valueMa.Value / 10;
		return WriteField<RegOffset::ChargeCurrentLimit>(value, 0, 9);
	}

	Error::Api::Result<MilliVolts> Device::GetDynamicPowerManagementInputVoltageLimit() const
	{
		auto voltage = ReadField<RegOffset::InputVoltageLimit>(0, 8);
		if (!voltage.has_value())
		{
			return std::unexpected(voltage.error());
		}
		return MilliVolts(voltage.value()) * 100_mV;
	}

	Error::Api::Result<void> Device::SetDynamicPowerManagementInputVoltageLimit(MilliVolts valueMv) const
	{
		uint16_t value = valueMv.Value / 100;
		return WriteField<RegOffset::InputVoltageLimit>(value, 0, 8);
	}

	Error::Api::Result<MilliAmperes> Device::GetDynamicPowerManagementInputCurrentLimit() const
	{
		auto value = ReadField<RegOffset::InputCurrentLimit>(0, 9);
		if (!value.has_value())
		{
			return std::unexpected(value.error());
		}
		return MilliAmperes(value.value()) * 10_mA;
	}

	Error::Api::Result<void> Device::SetDynamicPowerManagementInputCurrentLimit(MilliAmperes valueMa) const
	{
		uint16_t value = valueMa.Value / 10;
		return WriteField<RegOffset::InputCurrentLimit>(value, 0, 9);
	}

	Error::Api::Result<PrechargeControl> Device::GetPrechargeControl() const
	{
		uint8_t regValue;
		if (auto status = Read(RegOffset::PrechargeControl, {&regValue, 1}); !status.has_value())
		{
			return std::unexpected(status.error());
		}

		PrechargeControl prechargeControl{};
		auto currentLimitRaw = RegUtils::ReadInt<uint8_t>(&regValue, 0, 6);
		prechargeControl.CurrentLimit = MilliAmperes(currentLimitRaw) * 40_mA;
		prechargeControl.VoltageThreshold = RegUtils::ReadEnum<FastChargeVoltageThreshold>(&regValue, 6, 2);
		return prechargeControl;
	}

	Error::Api::Result<void> Device::SetPrechargeControl(const PrechargeControl& value) const
	{
		uint8_t regValue = 0;
		RegUtils::Write(value.VoltageThreshold, &regValue, 6, 2);
		uint8_t currentLimitRaw = value.CurrentLimit.Value / 40;
		RegUtils::Write(currentLimitRaw, &regValue, 0, 6);
		return Write(RegOffset::PrechargeControl, &regValue, 1);
	}

	Error::Api::Result<void> Device::Reset() const
	{
		return WriteField<RegOffset::TerminationControl>(1, 6, 1);
	}

	Error::Api::Result<MilliAmperes> Device::GetTerminationCurrent() const
	{
		auto field = ReadField<RegOffset::TerminationControl>(0, 5);
		if (!field.has_value())
		{
			return std::unexpected(field.error());
		}
		return MilliAmperes(field.value()) * 40_mA;
	}

	Error::Api::Result<void> Device::SetTerminationCurrent(MilliAmperes valueMa) const
	{
		uint8_t value = valueMa.Value / 40;
		return WriteField<RegOffset::TerminationControl>(value, 0, 5);
	}

	Error::Api::Result<Cells> Device::GetCells() const
	{
		auto field = ReadField<RegOffset::RechargeControl>(6, 2);
		if (!field.has_value())
		{
			return std::unexpected(field.error());
		}
		return static_cast<Cells>(field.value());
	}

	Error::Api::Result<void> Device::SetCells(const Cells& value) const
	{
		return WriteField<RegOffset::RechargeControl>(value, 6, 2);
	}

	Error::Api::Result<RechargeDeglichTime> Device::GetRechargeDeglichTime() const
	{
		return ReadFieldEnum<RegOffset::RechargeControl, RechargeDeglichTime>(4, 2);
	}

	Error::Api::Result<void> Device::SetCells(const RechargeDeglichTime& value) const
	{
		return WriteFieldEnum<RegOffset::RechargeControl>(value, 4, 2);
	}

	Error::Api::Result<MilliVolts> Device::GetRechargeThresholdOffset() const
	{
		return ReadFieldUnit<RegOffset::RechargeControl>(0, 4, 50_mV, 50_mV);
	}

	Error::Api::Result<void> Device::SetRechargeThresholdOffset(MilliVolts valueMv) const
	{
		return WriteFieldUnit<RegOffset::RechargeControl>(0, 4, valueMv, 50_mV, 50_mV);
	}

	Error::Api::Result<MilliVolts> Device::GetOtgRegulationVoltage() const
	{
		return ReadFieldUnit<RegOffset::VotgRegulation>(0, 11, 2800_mV, 10_mV);
	}

	Error::Api::Result<void> Device::SetOtgRegulationVoltage(MilliVolts valueMv) const
	{
		return WriteFieldUnit<RegOffset::VotgRegulation>(0, 11, valueMv, 2800_mV, 10_mV);
	}

	Error::Api::Result<PrechargeSafetyTimer> Device::GetPrechargeSafetyTimer() const
	{
		return ReadFieldEnum<RegOffset::IotgRegulation, PrechargeSafetyTimer>(7, 1);
	}

	Error::Api::Result<void> Device::SetPrechargeSafetyTimer(PrechargeSafetyTimer value) const
	{
		return WriteFieldEnum<RegOffset::RechargeControl>(value, 7, 1);
	}

	Error::Api::Result<MilliAmperes> Device::GetOtgCurrentLimit() const
	{
		return ReadFieldUnit<RegOffset::IotgRegulation>(0, 7, 0_mA, 40_mA);
	}

	Error::Api::Result<void> Device::SetOtgCurrentLimit(MilliAmperes valueMa) const
	{
		return WriteFieldUnit<RegOffset::IotgRegulation>(0, 7, valueMa, 0_mA, 40_mA);
	}

	Error::Api::Result<ThermalRegulation> Device::GetThermalRegulationThreshold() const
	{
		return ReadFieldEnum<RegOffset::TemperatureControl, ThermalRegulation>(6, 2);
	}

	Error::Api::Result<void> Device::SetThermalRegulationThreshold(const ThermalRegulation& value) const
	{
		return WriteFieldEnum<RegOffset::TemperatureControl>(value, 6, 2);
	}

	Error::Api::Result<ThermalShutdown> Device::GetThermalShutdownThreshold() const
	{
		return ReadFieldEnum<RegOffset::TemperatureControl, ThermalShutdown>(4, 2);
	}

	Error::Api::Result<void> Device::SetThermalShutdownThreshold(const ThermalShutdown& value) const
	{
		return WriteFieldEnum<RegOffset::TemperatureControl>(value, 4, 2);
	}

	Error::Api::Result<void> Device::SetTsIgnore(bool value) const
	{
		uint8_t value8 = value ? 1 : 0;
		return WriteField<RegOffset::NtcControl1>(value8, 0, 1);
	}

	Error::Api::Result<bool> Device::GetTsIgnore() const
	{
		auto field = ReadField<RegOffset::NtcControl1>(0, 1);
		if (field.has_value()) return field.value() != 0;
		return std::unexpected(field.error());
	}

	Error::Api::Result<IbatReg> Device::GetOtgMaxCurrent() const
	{
		auto field = ReadField<RegOffset::ChargerControl5>(3, 2);
		if (field.has_value()) return static_cast<IbatReg>(field.value());
		return std::unexpected(field.error());
	}

	Error::Api::Result<void> Device::SetOtgMaxCurrent(IbatReg value) const
	{
		return WriteField<RegOffset::ChargerControl5>(static_cast<uint8_t>(value), 3, 2);
	}

	Error::Api::Result<bool> Device::IsSfetPresent() const
	{
		auto field = ReadField<RegOffset::ChargerControl5>(7, 1);
		if (field.has_value()) return field.value() != 0;
		return std::unexpected(field.error());
	}

	Error::Api::Result<void> Device::SetSfetPresent(bool value) const
	{
		return WriteField<RegOffset::ChargerControl5>(value ? 1 : 0, 7, 1);
	}

	Error::Api::Result<bool> Device::IsDischargeCurrentSensingEnabled() const
	{
		auto field = ReadField<RegOffset::ChargerControl5>(5, 1);
		if (field.has_value()) return field.value() != 0;
		return std::unexpected(field.error());
	}

	Error::Api::Result<void> Device::SetDischargeCurrentSensingEnabled(bool value) const
	{
		return WriteField<RegOffset::ChargerControl5>(value ? 1 : 0, 5, 1);
	}

	Error::Api::Result<bool> Device::IsIlimHizCurrentLimitEnabled() const
	{
		auto field = ReadField<RegOffset::ChargerControl5>(1, 1);
		if (field.has_value()) return field.value() != 0;
		return std::unexpected(field.error());
	}

	Error::Api::Result<void> Device::SetIlimHizCurrentLimitEnabled(bool value) const
	{
		return WriteField<RegOffset::ChargerControl5>(value ? 1 : 0, 1, 1);
	}

	Error::Api::Result<bool> Device::IsDischargeOcpEnabled() const
	{
		auto field = ReadField<RegOffset::ChargerControl5>(0, 1);
		if (field.has_value()) return field.value() != 0;
		return std::unexpected(field.error());
	}

	Error::Api::Result<void> Device::SetDischargeOcpEnabled(bool value) const
	{
		return WriteField<RegOffset::ChargerControl5>(value ? 1 : 0, 0, 1);
	}

	Error::Api::Result<bool> Device::GetWdRst() const
	{
		auto field = ReadField<RegOffset::ChargerControl1>(3, 1);
		if (field.has_value()) return field.value() != 0;
		return std::unexpected(field.error());
	}

	Error::Api::Result<void> Device::SetWdRst(bool value) const
	{
		return WriteField<RegOffset::ChargerControl1>(value ? 1 : 0, 3, 1);
	}

	Error::Api::Result<Watchdog> Device::GetWatchdog() const
	{
		auto field = ReadField<RegOffset::ChargerControl1>(0, 3);
		if (field.has_value()) return static_cast<Watchdog>(field.value());
		return std::unexpected(field.error());
	}

	Error::Api::Result<void> Device::SetWatchdog(Watchdog value) const
	{
		return WriteField<RegOffset::ChargerControl1>(static_cast<uint8_t>(value), 0, 3);
	}

	Error::Api::Result<VacOvp> Device::GetVacOvervoltageThreshold() const
	{
		return ReadFieldEnum<RegOffset::ChargerControl1, VacOvp>(4, 2);
	}

	Error::Api::Result<void> Device::SetVacOvervoltageThreshold(VacOvp value) const
	{
		return WriteFieldEnum<RegOffset::ChargerControl1, VacOvp>(value, 4, 2);
	}

	Error::Api::Result<AdcSpeed> Device::GetAdcSampleSpeed() const
	{
		auto field = ReadField<RegOffset::AdcControl>(4, 2);
		if (field.has_value()) return static_cast<AdcSpeed>(field.value());
		return std::unexpected(field.error());
	}

	Error::Api::Result<void> Device::SetAdcSampleSpeed(AdcSpeed value) const
	{
		return WriteField<RegOffset::AdcControl>(static_cast<uint8_t>(value), 4, 2);
	}

	Error::Api::Result<bool> Device::IsAdcEnabled() const
	{
		auto field = ReadField<RegOffset::AdcControl>(7, 1);
		if (field.has_value()) return field.value() != 0;
		return std::unexpected(field.error());
	}

	Error::Api::Result<void> Device::SetAdcEnabled(bool value) const
	{
		return WriteField<RegOffset::AdcControl>(value ? 1 : 0, 7, 1);
	}

	Error::Api::Result<ChargerStatus0Flags> Device::GetChargerStatus0() const
	{
		auto field = ReadField<RegOffset::ChargerStatus0>(0, 8);
		if (field.has_value()) return static_cast<ChargerStatus0Flags>(field.value());
		return std::unexpected(field.error());
	}

	Error::Api::Result<ChargeStatus> Device::GetChargeStatus() const
	{
		auto field = ReadField<RegOffset::ChargerStatus1>(5, 3);
		if (field.has_value()) return static_cast<ChargeStatus>(field.value());
		return std::unexpected(field.error());
	}

	Error::Api::Result<VbusStatus> Device::GetVbusStatus() const
	{
		auto field = ReadField<RegOffset::ChargerStatus1>(1, 4);
		if (field.has_value()) return static_cast<VbusStatus>(field.value());
		return std::unexpected(field.error());
	}

	Error::Api::Result<bool> Device::IsBc12DetectionComplete() const
	{
		auto field = ReadField<RegOffset::ChargerStatus1>(0, 1);
		if (field.has_value()) return field.value() != 0;
		return std::unexpected(field.error());
	}

	Error::Api::Result<IcoStatus> Device::GetIcoStatus() const
	{
		auto field = ReadField<RegOffset::ChargerStatus2>(6, 2);
		if (field.has_value()) return static_cast<IcoStatus>(field.value());
		return std::unexpected(field.error());
	}

	Error::Api::Result<bool> Device::IsInThermalRegulation() const
	{
		auto field = ReadField<RegOffset::ChargerStatus2>(2, 1);
		if (field.has_value()) return field.value() != 0;
		return std::unexpected(field.error());
	}

	Error::Api::Result<bool> Device::IsDpDmDetectionOngoing() const
	{
		auto field = ReadField<RegOffset::ChargerStatus2>(1, 1);
		if (field.has_value()) return field.value() != 0;
		return std::unexpected(field.error());
	}

	Error::Api::Result<bool> Device::IsBatteryPresent() const
	{
		auto field = ReadField<RegOffset::ChargerStatus2>(0, 1);
		if (field.has_value()) return field.value() != 0;
		return std::unexpected(field.error());
	}

	Error::Api::Result<Fault0> Device::GetFault0() const
	{
		return ReadFieldEnum<RegOffset::FaultStatus0, Fault0>(0, 8);
	}

	Error::Api::Result<Fault1> Device::GetFault1() const
	{
		return ReadFieldEnum<RegOffset::FaultStatus1, Fault1>(0, 8);
	}

	Error::Api::Result<MilliAmperes> Device::GetIbusCurrent() const
	{
		auto field = ReadField<RegOffset::IbusAdc>(0, 16);
		if (field.has_value()) return MilliAmperes(static_cast<int16_t>(field.value()));
		return std::unexpected(field.error());
	}

	Error::Api::Result<MilliAmperes> Device::GetIbatCurrent() const
	{
		auto field = ReadField<RegOffset::IbatAdc>(0, 16);
		if (field.has_value()) return MilliAmperes(static_cast<int16_t>(field.value()));
		return std::unexpected(field.error());
	}

	Error::Api::Result<MilliVolts> Device::GetVbusVoltage() const
	{
		auto field = ReadField<RegOffset::VbusAdc>(0, 16);
		if (field.has_value()) return MilliVolts(field.value());
		return std::unexpected(field.error());
	}

	Error::Api::Result<MilliVolts> Device::GetVbatVoltage() const
	{
		auto field = ReadField<RegOffset::VbatAdc>(0, 16);
		if (field.has_value()) return MilliVolts(field.value());
		return std::unexpected(field.error());
	}

	Error::Api::Result<MilliVolts> Device::GetVsysVoltage() const
	{
		auto field = ReadField<RegOffset::VsysAdc>(0, 16);
		if (field.has_value()) return MilliVolts(field.value());
		return std::unexpected(field.error());
	}

	Error::Api::Result<NormalizedIntFraction<16>> Device::GetTsPercentage() const
	{
		auto field = ReadField<RegOffset::TsAdc>(0, 16);
		if (field.has_value()) return NormalizedIntFraction<16>(field.value());
		return std::unexpected(field.error());
	}

	Error::Api::Result<Celcius> Device::GetDieTemperature() const
	{
		auto field = ReadField<RegOffset::TdieAdc>(0, 16);

		if (!field.has_value())
		{
			return std::unexpected(field.error());
		}

		auto rawHalves = static_cast<int16_t>(field.value());

		return Celcius(rawHalves);
	}

	Error::Api::Result<MilliVolts> Device::GetUsbDataPlusVoltage() const
	{
		auto field = ReadField<RegOffset::DpAdc>(0, 16);
		if (field.has_value()) return MilliVolts(field.value());
		return std::unexpected(field.error());
	}

	Error::Api::Result<MilliVolts> Device::GetUsbDataMinusVoltage() const
	{
		auto field = ReadField<RegOffset::DmAdc>(0, 16);
		if (field.has_value()) return MilliVolts(field.value());
		return std::unexpected(field.error());
	}

	Error::Api::Result<bool> Device::IsAutomaticDpDmDetectionEnabled() const
	{
		auto field = ReadField<RegOffset::ChargerControl2>(6, 1);
		if (field.has_value()) return field.value() != 0;
		return std::unexpected(field.error());
	}

	Error::Api::Result<void> Device::SetAutomaticDpDmDetectionEnabled(bool value) const
	{
		return WriteField<RegOffset::ChargerControl2>(value ? 1 : 0, 6, 1);
	}

	Error::Api::Result<void> Device::ForceDpDmDetection() const
	{
		return WriteField<RegOffset::ChargerControl2>(1, 7, 1);
	}

	Error::Api::Result<DpDac> Device::GetDpDac() const
	{
		return ReadFieldEnum<RegOffset::DpDmDriver, DpDac>(5, 3);
	}

	Error::Api::Result<void> Device::SetDpDac(DpDac value) const
	{
		return WriteFieldEnum<RegOffset::DpDmDriver>(value, 5, 3);
	}

	Error::Api::Result<DmDac> Device::GetDmDac() const
	{
		return ReadFieldEnum<RegOffset::DpDmDriver, DmDac>(5, 3);
	}

	Error::Api::Result<void> Device::SetDmDac(DmDac value) const
	{
		return WriteFieldEnum<RegOffset::DpDmDriver>(value, 5, 3);
	}

    Error::Api::Result<void> Device::Read(uint8_t offset, std::span<uint8_t> data) const
    {
        auto writeResult = m_Driver.Write(Address, std::span<const uint8_t>(&offset, 1));
        if (!writeResult.has_value())
        {
            return std::unexpected(writeResult.error());
        }

        return m_Driver.Read(Address, data);
    }

	Error::Api::Result<void> Device::Write(uint8_t offset, uint8_t* data, size_t size) const
	{
		std::vector<uint8_t> buffer;
		buffer.resize(size + 1);
		buffer[0] = offset;
		std::memcpy(buffer.data() + 1, data, size);

        return m_Driver.Write(Address, std::span<const uint8_t>(buffer.data(), buffer.size()));
    }
}
