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

    ProtocolError Device::SetTsIgnore(bool value)
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