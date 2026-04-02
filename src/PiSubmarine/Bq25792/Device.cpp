#include <utility>

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
        return ReadField<RegOffset::NtcControl1>(0, 1);
    }

    /*

    IbatReg Device::GetOtgMaxCurrent() const
    {
        return RegUtils::Read<IbatReg, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerControl5), 3, 2);
    }

    void Device::SetOtgMaxCurrent(IbatReg value)
    {
        RegUtils::Write<IbatReg, std::endian::big>(value, m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerControl5), 0, 1);
        m_DirtyRegs[RegUtils::ToInt(RegOffset::ChargerControl5)] = true;
    }

    bool Device::IsSfetPresent() const
    {
        return RegUtils::Read<uint8_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerControl5), 7, 1);
    }

    void Device::SetSfetPresent(bool value)
    {
        RegUtils::Write<uint8_t, std::endian::big>(value, m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerControl5), 7, 1);
        m_DirtyRegs[RegUtils::ToInt(RegOffset::ChargerControl5)] = true;
    }

    bool Device::IsDischargeCurrentSensingEnabled() const
    {
        return RegUtils::Read<uint8_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerControl5), 5, 1);
    }

    void Device::SetDischargeCurrentSensingEnabled(bool value)
    {
        RegUtils::Write<uint8_t, std::endian::big>(value, m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerControl5), 5, 1);
        m_DirtyRegs[RegUtils::ToInt(RegOffset::ChargerControl5)] = true;
    }

    bool Device::IsIlimHizCurrentLimitEnabled() const
    {
        return RegUtils::Read<uint8_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerControl5), 1, 1);
    }

    void Device::SetIlimHizCurrentLimitEnabled(bool value)
    {
        RegUtils::Write<uint8_t, std::endian::big>(value, m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerControl5), 1, 1);
        m_DirtyRegs[RegUtils::ToInt(RegOffset::ChargerControl5)] = true;
    }

    bool Device::IsDischargeOcpEnabled() const
    {
        return RegUtils::Read<uint8_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerControl5), 0, 1);
    }

    void Device::SetDischargeOcpEnabled(bool value)
    {
        RegUtils::Write<uint8_t, std::endian::big>(value, m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerControl5), 0, 1);
        m_DirtyRegs[RegUtils::ToInt(RegOffset::ChargerControl5)] = true;
    }

    void Device::SetWdRst(bool value)
    {
        RegUtils::Write<uint8_t, std::endian::big>(value, m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerControl1), 3, 1);
        m_DirtyRegs[RegUtils::ToInt(RegOffset::ChargerControl1)] = true;
    }

    bool Device::GetWdRst() const
    {
        return RegUtils::Read<uint8_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerControl1), 3, 1);
    }

    void Device::SetWatchdog(Watchdog value)
    {
        RegUtils::Write<Watchdog, std::endian::big>(value, m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerControl1), 3, 1);
        m_DirtyRegs[RegUtils::ToInt(RegOffset::ChargerControl1)] = true;
    }

    Watchdog Device::GetWatchdog() const
    {
        return RegUtils::Read<Watchdog, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerControl1), 0, 3);
    }

    AdcSpeed Device::GetAdcSampleSpeed() const
    {
        return RegUtils::Read<AdcSpeed, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::AdcControl), 4, 2);
    }

    void Device::SetAdcSampleSpeed(AdcSpeed value)
    {
        RegUtils::Write<AdcSpeed, std::endian::big>(value, m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::AdcControl), 4, 2);
        m_DirtyRegs[RegUtils::ToInt(RegOffset::ChargerControl1)] = true;
    }

    bool Device::IsAdcEnabled() const
    {
        return RegUtils::Read<uint8_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::AdcControl), 7, 1);
    }

    void Device::SetAdcEnabled(bool value)
    {
        RegUtils::Write<uint8_t, std::endian::big>(value, m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::AdcControl), 7, 1);
        m_DirtyRegs[RegUtils::ToInt(RegOffset::AdcControl)] = true;
    }

    ChargerStatus0Flags Device::GetChargerStatus0() const
    {
        return RegUtils::Read<ChargerStatus0Flags, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerStatus0), 0, 8);
    }

    ChargeStatus Device::GetChargeStatus() const
    {
        return RegUtils::Read<ChargeStatus, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerStatus1), 5, 3);
    }

    VbusStatus Device::GetVbusStatus() const
    {
        return RegUtils::Read<VbusStatus, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerStatus1), 1, 4);
    }

    bool Device::IsBc12DetectionComplete() const
    {
        return RegUtils::Read<uint8_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerStatus1), 0, 1);
    }

    IcoStatus Device::GetIcoStatus() const
    {
        return RegUtils::Read<IcoStatus, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerStatus2), 6, 2);
    }

    bool Device::IsInThermalRegulation() const
    {
        return RegUtils::Read<uint8_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerStatus2), 2, 1);
    }

    bool Device::IsDpDmDetectionOngoing() const
    {
        return RegUtils::Read<uint8_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerStatus2), 1, 1);
    }

    bool Device::IsBatteryPresent() const
    {
        return RegUtils::Read<uint8_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerStatus2), 0, 1);
    }

    MilliAmperes Device::GetIbusCurrent() const
    {
        auto isubAdc = RegUtils::Read<int16_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::IbusAdc), 0, 16);
        return MilliAmperes(isubAdc);
    }

    MilliAmperes Device::GetIbatCurrent() const
    {
        auto value = RegUtils::Read<int16_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::IbatAdc), 0, 16);
        return MilliAmperes(value);
    }

    MilliVolts Device::GetVbusVoltage() const
    {
        auto value = RegUtils::Read<uint16_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::VbusAdc), 0, 16);
        return MilliVolts(value);
    }

    MilliVolts Device::GetVbatVoltage() const
    {
        auto value = RegUtils::Read<uint16_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::VbatAdc), 0, 16);
        return MilliVolts(value);
    }

    MilliVolts Device::GetVsysVoltage() const
    {
        auto value = RegUtils::Read<uint16_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::VsysAdc), 0, 16);
        return MilliVolts(value);
    }

    NormalizedIntFraction<16> Device::GetTsPercentage() const
    {
        auto value = RegUtils::Read<uint16_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::TsAdc), 0, 16);
        return NormalizedIntFraction<16>(value);
    }

    Celcius Device::GetDieTemperature() const
    {
        auto value = RegUtils::Read<int16_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::TdieAdc), 0, 16);
        return Celcius(value);
    }

    MilliVolts Device::GetUsbDataPlusVoltage() const
    {
        auto value = RegUtils::Read<uint16_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::DpAdc), 0, 16);
        return MilliVolts(value);
    }

    MilliVolts Device::GetUsbDataMinusVoltage() const
    {
        auto value = RegUtils::Read<uint16_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::DmAdc), 0, 16);
        return MilliVolts(value);
    }

    bool Device::IsAutomaticDpDmDetectionEnabled() const
    {
        auto value = RegUtils::Read<uint8_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerControl2), 6, 1);
        return value;
    }

    void Device::SetAutomaticDpDmDetectionEnabled(bool value)
    {
        RegUtils::Write<uint8_t, std::endian::big>(value, m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargerControl2), 6, 1);
        m_DirtyRegs[RegUtils::ToInt(RegOffset::ChargerControl2)] = true;
    }
*/

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
        memcpy(buffer.data() + 1, data, size);

        return m_Driver.Write(Address, buffer.data(), buffer.size()) ? ProtocolError::Ok : ProtocolError::WriteError;
    }
}
