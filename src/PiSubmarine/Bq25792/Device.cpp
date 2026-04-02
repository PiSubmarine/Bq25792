#include <utility>

#include "PiSubmarine/Bq25792/Device.h"

namespace PiSubmarine::Bq25792
{
    Device::Device(PiSubmarine::I2C::Api::IDriverAsync& driver): m_Driver(driver)
    {

    }

    bool Device::IsTransactionInProgress() const
    {
        return m_IsTransactionInProgress;
    }

    bool Device::HasError() const
    {
        return m_HasError;
    }

    bool Device::WaitForTransaction(const WaitFunc& waitFunc) const
    {
        while (IsTransactionInProgress())
        {
            waitFunc(std::chrono::milliseconds(10));
        }
        return !HasError();
    }

    bool Device::Read()
    {
        std::bitset<MemorySize> regs;
        regs.set();
        return Read(0, m_ChargerMemoryBuffer.data(), m_ChargerMemoryBuffer.size(), regs);
    }

    bool Device::ReadAndWait(const WaitFunc& waitFunc)
    {
        if (!Read())
        {
            return false;
        }

        return WaitForTransaction(waitFunc);
    }

    bool Device::ReadAndWait(RegOffset reg, const WaitFunc& waitFunc)
    {
        if (!Read(reg))
        {
            return false;
        }

        return WaitForTransaction(waitFunc);
    }

    bool Device::Read(RegOffset reg)
    {
        std::bitset<MemorySize> regs;
        regs.set(RegUtils::ToInt(reg));
        size_t regSize = GetRegisterSize(reg);
        return Read(static_cast<uint8_t>(reg), m_ChargerMemoryBuffer.data() + static_cast<size_t>(reg), regSize, regs);
    }

    bool Device::Write()
    {
        std::bitset<MemorySize> regs;
        regs.set();
        return Write(0, m_ChargerMemoryBuffer.data(), m_ChargerMemoryBuffer.size(), regs);
    }

    bool Device::Write(RegOffset reg)
    {
        size_t regSize = GetRegisterSize(reg);
        std::bitset<MemorySize> regs;
        regs.set(RegUtils::ToInt(reg));
        return Write(static_cast<uint8_t>(reg), m_ChargerMemoryBuffer.data() + static_cast<size_t>(reg), regSize, regs);
    }

    bool Device::WriteAndWait(RegOffset reg, const WaitFunc& waitFunc)
    {
        if (!Write(reg))
        {
            return false;
        }

        return WaitForTransaction(waitFunc);
    }

    bool Device::WriteDirty()
    {
        if (m_IsTransactionInProgress)
        {
            return false;
        }

        m_HasError = false;
        m_IsTransactionInProgress = true;

        return WriteDirtyInternal(RegOffset{0});
    }

    bool Device::HasDirtyRegisters() const
    {
        return m_DirtyRegs.any();
    }

    MilliVolts Device::GetMinimalSystemVoltage() const
    {
        auto vsysMin = RegUtils::Read<uint8_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::MinimalSystemVoltage), 0, 6);
        return 2500_mV + MilliVolts(vsysMin) * 250_mV;
    }

    void Device::SetMinimalSystemVoltage(MilliVolts valueMv)
    {
        uint8_t value = (valueMv.Value - 2500) / 250;
        RegUtils::Write<uint8_t, std::endian::big>(value, m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::MinimalSystemVoltage), 0, 6);
        m_DirtyRegs[RegUtils::ToInt(RegOffset::MinimalSystemVoltage)] = true;
    }

    MilliAmperes Device::GetChargeCurrentLimit() const
    {
        auto Ichg = RegUtils::Read<uint16_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargeCurrentLimit), 0, 9);
        return MilliAmperes(Ichg) * 10_mA;
    }

    void Device::SetChargeCurrentLimit(MilliAmperes valueMa)
    {
        uint16_t value = valueMa.Value / 10;
        RegUtils::Write<uint16_t, std::endian::big>(value, m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::ChargeCurrentLimit), 0, 9);
        m_DirtyRegs[RegUtils::ToInt(RegOffset::ChargeCurrentLimit)] = true;
    }

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

    void Device::SetTsIgnore(bool value)
    {
        RegUtils::Write<uint8_t, std::endian::big>(value, m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::NtcControl1), 0, 1);
        m_DirtyRegs[RegUtils::ToInt(RegOffset::NtcControl1)] = true;
    }

    bool Device::GetTsIgnore() const
    {
        return RegUtils::Read<uint8_t, std::endian::big>(m_ChargerMemoryBuffer.data() + RegUtils::ToInt(RegOffset::NtcControl1), 0, 1);
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

    bool Device::Read(uint8_t offset, uint8_t* data, size_t size, const std::bitset<MemorySize>& regs)
    {
        if (m_IsTransactionInProgress)
        {
            return false;
        }

        auto writeCallback = [this, data, size, regs](uint8_t deviceAddress, bool success)
        {
            if (!success)
            {
                m_IsTransactionInProgress = false;
                return;
            }
            m_IsTransactionInProgress = m_Driver.ReadAsync(Address, data, size, [this, regs](uint8_t cbAddress, bool cbOk) {
                ReadCallback(cbAddress, regs, cbOk);
            });
        };

        bool writeStarted = m_Driver.WriteAsync(Address, &offset, 1, writeCallback);
        m_IsTransactionInProgress = writeStarted;
        m_HasError = !writeStarted;

        return writeStarted;
    }

    bool Device::Write(uint8_t offset, uint8_t* data, size_t size, const std::bitset<MemorySize>& regs)
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

    void Device::ReadCallback(uint8_t deviceAddress, std::bitset<MemorySize> regs, bool ok)
    {
        (void)deviceAddress;
        m_HasError = !ok;
        m_IsTransactionInProgress = false;

        if (ok)
        {
            m_DirtyRegs &= ~regs;
        }
    }

    void Device::WriteCallback(uint8_t deviceAddress, std::bitset<MemorySize> regs, bool ok)
    {
        (void)deviceAddress;
        m_HasError = !ok;
        m_IsTransactionInProgress = false;

        if (ok)
        {
            m_DirtyRegs &= ~regs;
        }
    }

    bool Device::WriteDirtyInternal(RegOffset regNext)
    {
        for (size_t i = RegUtils::ToInt(regNext); i < m_DirtyRegs.size(); i++)
        {
            if (!m_DirtyRegs[i])
            {
                continue;
            }
            auto reg = static_cast<RegOffset>(i);

            uint8_t regSize = GetRegisterSize(reg);
            std::vector<uint8_t> buffer;
            buffer.resize(regSize + 1);
            buffer[0] = i;
            memcpy(buffer.data() + 1, m_ChargerMemoryBuffer.data() + i, regSize);
            return m_Driver.WriteAsync(Address, buffer.data(), buffer.size(), [this, reg](uint8_t cbAddress, bool cbOk) {WriteDirtyCallback(cbAddress, reg, cbOk); });
        }
        return false;
    }

    void Device::WriteDirtyCallback(uint8_t deviceAddress, RegOffset reg, bool ok)
    {
        (void)deviceAddress;
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
}
