#include <utility>
#include <vector>
#include <cstring>

#include "PiSubmarine/Bq25792/Device.h"

namespace PiSubmarine::Bq25792
{
    Device::Device(PiSubmarine::I2C::Api::IDriver& driver) : m_Driver(driver)
    {
    }

    std::expected<MilliVolts, ProtocolError> Device::GetMinimalSystemVoltage() const
    {
        auto field = ReadField<RegOffset::MinimalSystemVoltage>(0, 6);
        if (field.has_value())
        {
            return 2500_mV + MilliVolts(field.value()) * 250_mV;
        }
        return std::unexpected(field.error());
    }

    ProtocolError Device::SetMinimalSystemVoltage(MilliVolts valueMv) const
    {
        uint8_t value = (valueMv.Value - 2500) / 250;
        return WriteField<RegOffset::MinimalSystemVoltage>(value, 0, 6);
    }

    std::expected<MilliVolts, ProtocolError> Device::GetChargeVoltageLimit() const
    {
        auto field = ReadField<RegOffset::ChargeVoltageLimit>(0, 11);
        if (field.has_value())
        {
            return MilliVolts(field.value()) * 10_mV;
        }
        return std::unexpected(field.error());
    }

    ProtocolError Device::SetChargeVoltageLimit(MilliVolts valueMv) const
    {
        uint16_t value = valueMv.Value / 10;
        return WriteField<RegOffset::ChargeVoltageLimit>(value, 0, 11);
    }

    std::expected<MilliAmperes, ProtocolError> Device::GetChargeCurrentLimit() const
    {
        auto current = ReadField<RegOffset::ChargeCurrentLimit>(0, 9);
        if (!current.has_value())
        {
            return std::unexpected(current.error());
        }
        return MilliAmperes(current.value()) * 10_mA;
    }


    ProtocolError Device::SetChargeCurrentLimit(MilliAmperes valueMa) const
    {
        uint16_t value = valueMa.Value / 10;
        return WriteField<RegOffset::ChargeCurrentLimit>(value, 0, 9);
    }

    std::expected<MilliVolts, ProtocolError> Device::GetDynamicPowerManagementInputVoltageLimit() const
    {
        auto voltage = ReadField<RegOffset::InputVoltageLimit>(0, 8);
        if (!voltage.has_value())
        {
            return std::unexpected(voltage.error());
        }
        return MilliVolts(voltage.value()) * 100_mV;
    }

    ProtocolError Device::SetDynamicPowerManagementInputVoltageLimit(MilliVolts valueMv) const
    {
        uint16_t value = valueMv.Value / 100;
        return WriteField<RegOffset::InputVoltageLimit>(value, 0, 8);
    }

    std::expected<MilliAmperes, ProtocolError> Device::GetDynamicPowerManagementInputCurrentLimit() const
    {
        auto value = ReadField<RegOffset::InputCurrentLimit>(0, 9);
        if (!value.has_value())
        {
            return std::unexpected(value.error());
        }
        return MilliAmperes(value.value()) * 10_mA;
    }

    ProtocolError Device::SetDynamicPowerManagementInputCurrentLimit(MilliAmperes valueMa) const
    {
        uint16_t value = valueMa.Value / 10;
        return WriteField<RegOffset::InputCurrentLimit>(value, 0, 9);
    }

    std::expected<PrechargeControl, ProtocolError> Device::GetPrechargeControl() const
    {
        uint8_t regValue;
        if (auto status = Read(RegOffset::PrechargeControl, &regValue, 1); status != ProtocolError::Ok)
        {
            return std::unexpected(status);
        }

        PrechargeControl prechargeControl{};
        auto currentLimitRaw = RegUtils::ReadInt<uint8_t>(&regValue, 0, 6);
        prechargeControl.CurrentLimit = MilliAmperes(currentLimitRaw) * 40_mA;
        prechargeControl.VoltageThreshold = RegUtils::ReadEnum<FastChargeVoltageThreshold>(&regValue, 6, 2);
        return prechargeControl;
    }

    ProtocolError Device::SetPrechargeControl(const PrechargeControl& value) const
    {
        uint8_t regValue = 0;
        RegUtils::Write(value.VoltageThreshold, &regValue, 6, 2);
        uint8_t currentLimitRaw = value.CurrentLimit.Value / 40;
        RegUtils::Write(currentLimitRaw, &regValue, 0, 6);
        return Write(RegOffset::PrechargeControl, &regValue, 1);
    }

    ProtocolError Device::Reset() const
    {
        return WriteField<RegOffset::TerminationControl>(1, 6, 1);
    }

    std::expected<MilliAmperes, ProtocolError> Device::GetTerminationCurrent() const
    {
        auto field = ReadField<RegOffset::TerminationControl>(0, 5);
        if (!field.has_value())
        {
            return std::unexpected(field.error());
        }
        return MilliAmperes(field.value()) * 40_mA;
    }

    ProtocolError Device::SetTerminationCurrent(MilliAmperes valueMa) const
    {
        uint8_t value = valueMa.Value / 40;
        return WriteField<RegOffset::TerminationControl>(value, 0, 5);
    }

    std::expected<Cells, ProtocolError> Device::GetCells() const
    {
        auto field = ReadField<RegOffset::RechargeControl>(6, 2);
        if (!field.has_value())
        {
            return std::unexpected(field.error());
        }
        return static_cast<Cells>(field.value());
    }

    ProtocolError Device::SetCells(const Cells& value) const
    {
        return WriteField<RegOffset::RechargeControl>(value, 6, 2);
    }

    std::expected<RechargeDeglichTime, ProtocolError> Device::GetRechargeDeglichTime() const
    {
        return ReadFieldEnum<RegOffset::RechargeControl, RechargeDeglichTime>(4, 2);
    }

    ProtocolError Device::SetCells(const RechargeDeglichTime& value) const
    {
        return WriteFieldEnum<RegOffset::RechargeControl>(value, 4, 2);
    }

    std::expected<MilliVolts, ProtocolError> Device::GetRechargeThresholdOffset() const
    {
        return ReadFieldUnit<RegOffset::RechargeControl>(0, 4, 50_mV, 50_mV);
    }

    ProtocolError Device::SetRechargeThresholdOffset(MilliVolts valueMv) const
    {
        return WriteFieldUnit<RegOffset::RechargeControl>(0, 4, valueMv, 50_mV, 50_mV);
    }

    std::expected<MilliVolts, ProtocolError> Device::GetOtgRegulationVoltage() const
    {
        return ReadFieldUnit<RegOffset::VotgRegulation>(0, 11, 2800_mV, 10_mV);
    }

    ProtocolError Device::SetOtgRegulationVoltage(MilliVolts valueMv) const
    {
        return WriteFieldUnit<RegOffset::VotgRegulation>(0, 11, valueMv, 2800_mV, 10_mV);
    }

    std::expected<PrechargeSafetyTimer, ProtocolError> Device::GetPrechargeSafetyTimer() const
    {
        return ReadFieldEnum<RegOffset::IotgRegulation, PrechargeSafetyTimer>(7, 1);
    }

    ProtocolError Device::SetPrechargeSafetyTimer(PrechargeSafetyTimer value) const
    {
        return WriteFieldEnum<RegOffset::RechargeControl>(value, 7, 1);
    }

    std::expected<MilliAmperes, ProtocolError> Device::GetOtgCurrentLimit() const
    {
        return ReadFieldUnit<RegOffset::IotgRegulation>(0, 7, 0_mA, 40_mA);
    }

    ProtocolError Device::SetOtgCurrentLimit(MilliAmperes valueMa) const
    {
        return WriteFieldUnit<RegOffset::IotgRegulation>(0, 7, valueMa, 0_mA, 40_mA);
    }

    std::expected<ThermalRegulation, ProtocolError> Device::GetThermalRegulationThreshold() const
    {
        return ReadFieldEnum<RegOffset::TemperatureControl, ThermalRegulation>(6, 2);
    }

    ProtocolError Device::SetThermalRegulationThreshold(const ThermalRegulation& value) const
    {
        return WriteFieldEnum<RegOffset::TemperatureControl>(value, 6, 2);
    }

    std::expected<ThermalShutdown, ProtocolError> Device::GetThermalShutdownThreshold() const
    {
        return ReadFieldEnum<RegOffset::TemperatureControl, ThermalShutdown>(4, 2);
    }

    ProtocolError Device::SetThermalShutdownThreshold(const ThermalShutdown& value) const
    {
        return WriteFieldEnum<RegOffset::TemperatureControl>(value, 4, 2);
    }

    ProtocolError Device::SetTsIgnore(bool value) const
    {
        uint8_t value8 = value ? 1 : 0;
        return WriteField<RegOffset::NtcControl1>(value8, 0, 1);
    }

    std::expected<bool, ProtocolError> Device::GetTsIgnore() const
    {
        auto field = ReadField<RegOffset::NtcControl1>(0, 1);
        if (field.has_value()) return field.value() != 0;
        return std::unexpected(field.error());
    }

    std::expected<IbatReg, ProtocolError> Device::GetOtgMaxCurrent() const
    {
        auto field = ReadField<RegOffset::ChargerControl5>(3, 2);
        if (field.has_value()) return static_cast<IbatReg>(field.value());
        return std::unexpected(field.error());
    }

    ProtocolError Device::SetOtgMaxCurrent(IbatReg value) const
    {
        return WriteField<RegOffset::ChargerControl5>(static_cast<uint8_t>(value), 3, 2);
    }

    std::expected<bool, ProtocolError> Device::IsSfetPresent() const
    {
        auto field = ReadField<RegOffset::ChargerControl5>(7, 1);
        if (field.has_value()) return field.value() != 0;
        return std::unexpected(field.error());
    }

    ProtocolError Device::SetSfetPresent(bool value) const
    {
        return WriteField<RegOffset::ChargerControl5>(value ? 1 : 0, 7, 1);
    }

    std::expected<bool, ProtocolError> Device::IsDischargeCurrentSensingEnabled() const
    {
        auto field = ReadField<RegOffset::ChargerControl5>(5, 1);
        if (field.has_value()) return field.value() != 0;
        return std::unexpected(field.error());
    }

    ProtocolError Device::SetDischargeCurrentSensingEnabled(bool value) const
    {
        return WriteField<RegOffset::ChargerControl5>(value ? 1 : 0, 5, 1);
    }

    std::expected<bool, ProtocolError> Device::IsIlimHizCurrentLimitEnabled() const
    {
        auto field = ReadField<RegOffset::ChargerControl5>(1, 1);
        if (field.has_value()) return field.value() != 0;
        return std::unexpected(field.error());
    }

    ProtocolError Device::SetIlimHizCurrentLimitEnabled(bool value) const
    {
        return WriteField<RegOffset::ChargerControl5>(value ? 1 : 0, 1, 1);
    }

    std::expected<bool, ProtocolError> Device::IsDischargeOcpEnabled() const
    {
        auto field = ReadField<RegOffset::ChargerControl5>(0, 1);
        if (field.has_value()) return field.value() != 0;
        return std::unexpected(field.error());
    }

    ProtocolError Device::SetDischargeOcpEnabled(bool value) const
    {
        return WriteField<RegOffset::ChargerControl5>(value ? 1 : 0, 0, 1);
    }

    std::expected<bool, ProtocolError> Device::GetWdRst() const
    {
        auto field = ReadField<RegOffset::ChargerControl1>(3, 1);
        if (field.has_value()) return field.value() != 0;
        return std::unexpected(field.error());
    }

    ProtocolError Device::SetWdRst(bool value) const
    {
        return WriteField<RegOffset::ChargerControl1>(value ? 1 : 0, 3, 1);
    }

    std::expected<Watchdog, ProtocolError> Device::GetWatchdog() const
    {
        auto field = ReadField<RegOffset::ChargerControl1>(0, 3);
        if (field.has_value()) return static_cast<Watchdog>(field.value());
        return std::unexpected(field.error());
    }

    ProtocolError Device::SetWatchdog(Watchdog value) const
    {
        return WriteField<RegOffset::ChargerControl1>(static_cast<uint8_t>(value), 0, 3);
    }

    std::expected<VacOvp, ProtocolError> Device::GetVacOvervoltageThreshold() const
    {
        return ReadFieldEnum<RegOffset::ChargerControl1, VacOvp>(4, 2);
    }

    ProtocolError Device::SetVacOvervoltageThreshold(VacOvp value) const
    {
        return WriteFieldEnum<RegOffset::ChargerControl1, VacOvp>(value, 4, 2);
    }

    std::expected<AdcSpeed, ProtocolError> Device::GetAdcSampleSpeed() const
    {
        auto field = ReadField<RegOffset::AdcControl>(4, 2);
        if (field.has_value()) return static_cast<AdcSpeed>(field.value());
        return std::unexpected(field.error());
    }

    ProtocolError Device::SetAdcSampleSpeed(AdcSpeed value) const
    {
        return WriteField<RegOffset::AdcControl>(static_cast<uint8_t>(value), 4, 2);
    }

    std::expected<bool, ProtocolError> Device::IsAdcEnabled() const
    {
        auto field = ReadField<RegOffset::AdcControl>(7, 1);
        if (field.has_value()) return field.value() != 0;
        return std::unexpected(field.error());
    }

    ProtocolError Device::SetAdcEnabled(bool value) const
    {
        return WriteField<RegOffset::AdcControl>(value ? 1 : 0, 7, 1);
    }

    std::expected<ChargerStatus0Flags, ProtocolError> Device::GetChargerStatus0() const
    {
        auto field = ReadField<RegOffset::ChargerStatus0>(0, 8);
        if (field.has_value()) return static_cast<ChargerStatus0Flags>(field.value());
        return std::unexpected(field.error());
    }

    std::expected<ChargeStatus, ProtocolError> Device::GetChargeStatus() const
    {
        auto field = ReadField<RegOffset::ChargerStatus1>(5, 3);
        if (field.has_value()) return static_cast<ChargeStatus>(field.value());
        return std::unexpected(field.error());
    }

    std::expected<VbusStatus, ProtocolError> Device::GetVbusStatus() const
    {
        auto field = ReadField<RegOffset::ChargerStatus1>(1, 4);
        if (field.has_value()) return static_cast<VbusStatus>(field.value());
        return std::unexpected(field.error());
    }

    std::expected<bool, ProtocolError> Device::IsBc12DetectionComplete() const
    {
        auto field = ReadField<RegOffset::ChargerStatus1>(0, 1);
        if (field.has_value()) return field.value() != 0;
        return std::unexpected(field.error());
    }

    std::expected<IcoStatus, ProtocolError> Device::GetIcoStatus() const
    {
        auto field = ReadField<RegOffset::ChargerStatus2>(6, 2);
        if (field.has_value()) return static_cast<IcoStatus>(field.value());
        return std::unexpected(field.error());
    }

    std::expected<bool, ProtocolError> Device::IsInThermalRegulation() const
    {
        auto field = ReadField<RegOffset::ChargerStatus2>(2, 1);
        if (field.has_value()) return field.value() != 0;
        return std::unexpected(field.error());
    }

    std::expected<bool, ProtocolError> Device::IsDpDmDetectionOngoing() const
    {
        auto field = ReadField<RegOffset::ChargerStatus2>(1, 1);
        if (field.has_value()) return field.value() != 0;
        return std::unexpected(field.error());
    }

    std::expected<bool, ProtocolError> Device::IsBatteryPresent() const
    {
        auto field = ReadField<RegOffset::ChargerStatus2>(0, 1);
        if (field.has_value()) return field.value() != 0;
        return std::unexpected(field.error());
    }

    std::expected<Fault0, ProtocolError> Device::GetFault0() const
    {
        return ReadFieldEnum<RegOffset::FaultStatus0, Fault0>(0, 8);
    }

    std::expected<Fault1, ProtocolError> Device::GetFault1() const
    {
        return ReadFieldEnum<RegOffset::FaultStatus1, Fault1>(0, 8);
    }

    std::expected<MilliAmperes, ProtocolError> Device::GetIbusCurrent() const
    {
        auto field = ReadField<RegOffset::IbusAdc>(0, 16);
        if (field.has_value()) return MilliAmperes(static_cast<int16_t>(field.value()));
        return std::unexpected(field.error());
    }

    std::expected<MilliAmperes, ProtocolError> Device::GetIbatCurrent() const
    {
        auto field = ReadField<RegOffset::IbatAdc>(0, 16);
        if (field.has_value()) return MilliAmperes(static_cast<int16_t>(field.value()));
        return std::unexpected(field.error());
    }

    std::expected<MilliVolts, ProtocolError> Device::GetVbusVoltage() const
    {
        auto field = ReadField<RegOffset::VbusAdc>(0, 16);
        if (field.has_value()) return MilliVolts(field.value());
        return std::unexpected(field.error());
    }

    std::expected<MilliVolts, ProtocolError> Device::GetVbatVoltage() const
    {
        auto field = ReadField<RegOffset::VbatAdc>(0, 16);
        if (field.has_value()) return MilliVolts(field.value());
        return std::unexpected(field.error());
    }

    std::expected<MilliVolts, ProtocolError> Device::GetVsysVoltage() const
    {
        auto field = ReadField<RegOffset::VsysAdc>(0, 16);
        if (field.has_value()) return MilliVolts(field.value());
        return std::unexpected(field.error());
    }

    std::expected<NormalizedIntFraction<16>, ProtocolError> Device::GetTsPercentage() const
    {
        auto field = ReadField<RegOffset::TsAdc>(0, 16);
        if (field.has_value()) return NormalizedIntFraction<16>(field.value());
        return std::unexpected(field.error());
    }

    std::expected<Celcius, ProtocolError> Device::GetDieTemperature() const
    {
        auto field = ReadField<RegOffset::TdieAdc>(0, 16);
        if (field.has_value()) return Celcius(static_cast<int16_t>(field.value()));
        return std::unexpected(field.error());
    }

    std::expected<MilliVolts, ProtocolError> Device::GetUsbDataPlusVoltage() const
    {
        auto field = ReadField<RegOffset::DpAdc>(0, 16);
        if (field.has_value()) return MilliVolts(field.value());
        return std::unexpected(field.error());
    }

    std::expected<MilliVolts, ProtocolError> Device::GetUsbDataMinusVoltage() const
    {
        auto field = ReadField<RegOffset::DmAdc>(0, 16);
        if (field.has_value()) return MilliVolts(field.value());
        return std::unexpected(field.error());
    }

    std::expected<bool, ProtocolError> Device::IsAutomaticDpDmDetectionEnabled() const
    {
        auto field = ReadField<RegOffset::ChargerControl2>(6, 1);
        if (field.has_value()) return field.value() != 0;
        return std::unexpected(field.error());
    }

    ProtocolError Device::SetAutomaticDpDmDetectionEnabled(bool value) const
    {
        return WriteField<RegOffset::ChargerControl2>(value ? 1 : 0, 6, 1);
    }

    ProtocolError Device::ForceDpDmDetection() const
    {
        return WriteField<RegOffset::ChargerControl2>(1, 7, 1);
    }

    std::expected<DpDac, ProtocolError> Device::GetDpDac() const
    {
        return ReadFieldEnum<RegOffset::DpDmDriver, DpDac>(5, 3);
    }

    ProtocolError Device::SetDpDac(DpDac value) const
    {
        return WriteFieldEnum<RegOffset::DpDmDriver>(value, 5, 3);
    }

    std::expected<DmDac, ProtocolError> Device::GetDmDac() const
    {
        return ReadFieldEnum<RegOffset::DpDmDriver, DmDac>(5, 3);
    }

    ProtocolError Device::StDmDac(DmDac value) const
    {
        return WriteFieldEnum<RegOffset::DpDmDriver>(value, 5, 3);
    }

    ProtocolError Device::Read(uint8_t offset, uint8_t* data, size_t size) const
    {
        if (!m_Driver.Write(Address, &offset, 1))
        {
            return ProtocolError::WriteError;
        }

        return m_Driver.Read(Address, data, size) ? ProtocolError::Ok : ProtocolError::ReadError;
    }

    ProtocolError Device::Write(uint8_t offset, uint8_t* data, size_t size) const
    {
        std::vector<uint8_t> buffer;
        buffer.resize(size + 1);
        buffer[0] = offset;
        std::memcpy(buffer.data() + 1, data, size);

        return m_Driver.Write(Address, buffer.data(), buffer.size()) ? ProtocolError::Ok : ProtocolError::WriteError;
    }
}