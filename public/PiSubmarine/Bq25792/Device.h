#pragma once

#include <functional>
#include <chrono>
#include <array>
#include "PiSubmarine/RegUtils.h"
#include "PiSubmarine/Bq25792/Units.h"
#include "PiSubmarine/NormalizedIntFraction.h"
#include "PiSubmarine/Error/Api/Result.h"
#include "PiSubmarine/I2C/Api/IDriver.h"

namespace PiSubmarine::Bq25792
{
    using WaitFunc = std::function<void(std::chrono::milliseconds)>;
    template <typename T>
    using Result = PiSubmarine::Error::Api::Result<T>;

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
        TimerControl = 0x0E,        // Not implemented
        ChargerControl0 = 0x0F,     // Not implemented
        ChargerControl1 = 0x10,
        ChargerControl2 = 0x11,     // Partially implemented
        ChargerControl3 = 0x12,     // Not implemented
        ChargerControl4 = 0x13,     // Not implemented
        ChargerControl5 = 0x14,     // Partially implemented
        TemperatureControl = 0x016, // Partially implemented
        NtcControl0 = 0x17,         // Not implemented
        NtcControl1 = 0x18,         // Partially implemented
        IcoCurrentLimit = 0x19,     // Not implemented
        ChargerStatus0 = 0x1B,
        ChargerStatus1 = 0x1C,
        ChargerStatus2 = 0x1D,
        ChargerStatus3 = 0x1E,      // Not implemented
        ChargerStatus4 = 0x1F,      // Not implemented
        FaultStatus0 = 0x20,
        FaultStatus1 = 0x21,
        ChargerFlag0 = 0x22,        // Not implemented
        ChargerFlag1 = 0x23,        // Not implemented
        ChargerFlag2 = 0x24,        // Not implemented
        ChargerFlag3 = 0x25,        // Not implemented
        FaultFlag0 = 0x26,          // Not implemented
        FaultFlag1 = 0x27,          // Not implemented
        ChargerMask0 = 0x28,        // Not implemented
        ChargerMask1 = 0x29,        // Not implemented
        ChargerMask2 = 0x2A,        // Not implemented
        ChargerMask3 = 0x2B,        // Not implemented
        FaultMask0 = 0x2C,          // Not implemented
        FaultMask1 = 0x2D,          // Not implemented
        AdcControl = 0x2E,          // Partially implemented
        AdcFunctionDisable0 = 0x2F, // Not implemented
        AdcFunctionDisable1 = 0x30, // Not implemented
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
        PartInformation = 0x48      // Not implemented
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
        (1ULL << static_cast<uint8_t>(RegOffset::VbatAdc)) |
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
        auto bitPos = static_cast<uint8_t>(reg);
        if (bitPos < 64)
        {
            return RegSizesA & (1ULL << bitPos) ? 2 : 1;
        }
        else
        {
            return RegSizesB & (1ULL << (bitPos - 64)) ? 2 : 1;
        }
    }

    template <size_t Bytes>
    struct RegisterTypeSelector
    {
        using type = void; // Default or error case
    };

    template <>
    struct RegisterTypeSelector<1>
    {
        using type = uint8_t;
    };

    template <>
    struct RegisterTypeSelector<2>
    {
        using type = uint16_t;
    };

    // Alias for convenience
    template <size_t Bytes>
    using RegisterType_t = typename RegisterTypeSelector<Bytes>::type;

    enum class FastChargeVoltageThreshold : uint8_t
    {
        Vreg15p,    // 15% x VREG
        Vreg62p2,   // 62.2% x VREG
        Vreg66p7,   // 66.7% x VREG
        Vreg71p4   // 71.4% x VREG
    };

    struct PrechargeControl
    {
        FastChargeVoltageThreshold VoltageThreshold;
        MilliAmperes CurrentLimit;
    };

    enum class Cells : uint8_t
    {
        Cells1,
        Cells2,
        Cells3,
        Cells4
    };

    enum class RechargeDeglichTime : uint8_t
    {
        Milliseconds64,
        Milliseconds256,
        Milliseconds1024,
        Milliseconds2048
    };

    enum class PrechargeSafetyTimer : uint8_t
    {
        Hours2,
        Minutes30
    };

    enum class ThermalRegulation : uint8_t
    {
        DegC60,
        DegC80,
        DegC100,
        DegC120
    };

    enum class ThermalShutdown : uint8_t
    {
        DegC150,
        DegC130,
        DegC120,
        DegC85
    };

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
        ChargingTerminationDone = 7
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

    enum class VacOvp
    {
        Volts26,
        Volts18,
        Volts12,
        Volts7
    };

    enum class Watchdog
    {
        Disable = 0,
        Sec0_5 = 1,
        Sec1 = 2,
        Sec2,
        Sec20,
        Sec40,
        Sec80,
        Sec160
    };

    enum class Fault0
    {
        Vac1Ovp = 1 << 0,
        Vac2Ovp = 1 << 1,
        ConvOcp = 1 << 2,
        IbatOcp = 1 << 3,
        IbusOcp = 1 << 4,
        VbatOvp = 1 << 5,
        VbusOvp = 1 << 6,
        IbatReg = 1 << 7,
    };

    enum class Fault1
    {
        TshutStat = 1 << 2,
        OtgUvp = 1 << 4,
        OtgOvp = 1 << 5,
        VsysOvp = 1 << 6,
        VsysShort = 1 << 7,
    };

    enum class AdcSpeed
    {
        Resolution15bits = 0,
        Resolution14bits,
        Resolution13bits,
        Resolution12bits
    };

    enum class DpDac
    {
        HiZ,
        Gnd,
        V0v6,
        V1v2,
        V2v2,
        V2v7,
        V3v3,
        DpDmShort
    };

    enum class DmDac
    {
        HiZ,
        Gnd,
        V0v6,
        V1v2,
        V2v2,
        V2v7,
        V3v3
    };

    class Device
    {
    public:
        constexpr static uint8_t Address = 0x6B;

        explicit Device(I2C::Api::IDriver& driver);

        /// <summary>
        /// Gets Minimal System Voltage (VSYSMIN)
        /// </summary>
        /// <returns>VSYSMIN in mV.</returns>
        [[nodiscard]] Result<MilliVolts> GetMinimalSystemVoltage() const;

        /// <summary>
        /// Sets minimal system voltage. Range: 2500mV - 16000mV, bit step size: 250mV
        /// </summary>
        /// <param name="valueMv">Voltage in mV</param>
        [[nodiscard]] Result<void> SetMinimalSystemVoltage(MilliVolts valueMv) const;


        /// <summary>
        /// Gets battery charge voltage limit.
        /// </summary>
        /// <returns>Charge voltage limit in mV</returns>
        [[nodiscard]] Result<MilliVolts> GetChargeVoltageLimit() const;


        /// <summary>
        /// Sets battery charge voltage limit.
        /// </summary>
        /// <param name="value">Charge voltage limit in mV. Range: 3000mV - 18800mV, bit step size: 10mV</param>
        /// <returns></returns>
        [[nodiscard]] Result<void> SetChargeVoltageLimit(MilliVolts value) const;

        /// <summary>
        /// Gets maximum charge current.
        /// </summary>
        /// <returns>Current in mA</returns>
        [[nodiscard]] Result<MilliAmperes> GetChargeCurrentLimit() const;

        /// <summary>
        /// Sets maximum charge current. Range: 50mA - 5000mA, bit step size: 10mA.
        /// </summary>
        /// <param name="valueMa">Current in mA</param>
        [[nodiscard]] Result<void> SetChargeCurrentLimit(MilliAmperes valueMa) const;


        /// <summary>
        /// Gets DPM Input Voltage Limit. See 9.3.8.2.
        /// </summary>
        /// <returns>VINDPM in mV</returns>
        [[nodiscard]] Result<MilliVolts> GetDynamicPowerManagementInputVoltageLimit() const;


        /// <summary>
        /// Sets DPM Input Voltage Limit. See 9.3.8.2.
        /// </summary>
        /// <param name="valueMv">Value in mV. Range: 3600mV - 22000mV. Bit step size: 100mV.</param>
        /// <returns>Operation result.</returns>
        [[nodiscard]] Result<void> SetDynamicPowerManagementInputVoltageLimit(MilliVolts valueMv) const;

        /// <summary>
        /// Gets DPM Input Current Limit. See 9.3.8.2.
        /// </summary>
        /// <returns>IINDPM in mA</returns>
        [[nodiscard]] Result<MilliAmperes> GetDynamicPowerManagementInputCurrentLimit() const;

        /// <summary>
        /// Sets DPM Input Current Limit. See 9.3.8.2.
        /// </summary>
        /// <param name="valueMa">Value in mA. Range: 100mA - 3300mA. Bit step size: 10mA.</param>
        /// <returns>Operation result.</returns>
        [[nodiscard]] Result<void> SetDynamicPowerManagementInputCurrentLimit(MilliAmperes valueMa) const;

        [[nodiscard]] Result<PrechargeControl> GetPrechargeControl() const;

        [[nodiscard]] Result<void> SetPrechargeControl(const PrechargeControl& value) const;

        [[nodiscard]] Result<void> Reset() const;

        [[nodiscard]] Result<MilliAmperes> GetTerminationCurrent() const;

        [[nodiscard]] Result<void> SetTerminationCurrent(MilliAmperes valueMa) const;

        [[nodiscard]] Result<Cells> GetCells() const;
        [[nodiscard]] Result<void> SetCells(const Cells& value) const;

        [[nodiscard]] Result<RechargeDeglichTime> GetRechargeDeglichTime() const;
        [[nodiscard]] Result<void> SetCells(const RechargeDeglichTime& value) const;

        [[nodiscard]] Result<MilliVolts> GetRechargeThresholdOffset() const;
        [[nodiscard]] Result<void> SetRechargeThresholdOffset(MilliVolts valueMv) const;

        [[nodiscard]] Result<MilliVolts> GetOtgRegulationVoltage() const;
        [[nodiscard]] Result<void> SetOtgRegulationVoltage(MilliVolts valueMv) const;

        [[nodiscard]] Result<PrechargeSafetyTimer> GetPrechargeSafetyTimer() const;
        [[nodiscard]] Result<void> SetPrechargeSafetyTimer(PrechargeSafetyTimer value) const;

        [[nodiscard]] Result<MilliAmperes> GetOtgCurrentLimit() const;
        [[nodiscard]] Result<void> SetOtgCurrentLimit(MilliAmperes valueMa) const;

        [[nodiscard]] Result<ThermalRegulation> GetThermalRegulationThreshold() const;
        [[nodiscard]] Result<void> SetThermalRegulationThreshold(const ThermalRegulation& value) const;

        [[nodiscard]] Result<ThermalShutdown> GetThermalShutdownThreshold() const;
        [[nodiscard]] Result<void> SetThermalShutdownThreshold(const ThermalShutdown& value) const;

        [[nodiscard]] Result<void> SetTsIgnore(bool value) const;
        [[nodiscard]] Result<bool> GetTsIgnore() const;

        [[nodiscard]] Result<IbatReg> GetOtgMaxCurrent() const;
        [[nodiscard]] Result<void> SetOtgMaxCurrent(IbatReg value) const;

        [[nodiscard]] Result<bool> IsSfetPresent() const;
        [[nodiscard]] Result<void> SetSfetPresent(bool value) const;

        [[nodiscard]] Result<bool> IsDischargeCurrentSensingEnabled() const;
        [[nodiscard]] Result<void> SetDischargeCurrentSensingEnabled(bool value) const;

        [[nodiscard]] Result<bool> IsIlimHizCurrentLimitEnabled() const;
        [[nodiscard]] Result<void> SetIlimHizCurrentLimitEnabled(bool value) const;

        [[nodiscard]] Result<bool> IsDischargeOcpEnabled() const;
        [[nodiscard]] Result<void> SetDischargeOcpEnabled(bool value) const;

        [[nodiscard]] Result<bool> GetWdRst() const;
        [[nodiscard]] Result<void> SetWdRst(bool value) const;

        [[nodiscard]] Result<Watchdog> GetWatchdog() const;
        [[nodiscard]] Result<void> SetWatchdog(Watchdog value) const;

        [[nodiscard]] Result<VacOvp> GetVacOvervoltageThreshold() const;
        [[nodiscard]] Result<void> SetVacOvervoltageThreshold(VacOvp value) const;

        [[nodiscard]] Result<AdcSpeed> GetAdcSampleSpeed() const;
        [[nodiscard]] Result<void> SetAdcSampleSpeed(AdcSpeed value) const;

        [[nodiscard]] Result<bool> IsAdcEnabled() const;
        [[nodiscard]] Result<void> SetAdcEnabled(bool value) const;

        [[nodiscard]] Result<ChargerStatus0Flags> GetChargerStatus0() const;
        [[nodiscard]] Result<ChargeStatus> GetChargeStatus() const;
        [[nodiscard]] Result<VbusStatus> GetVbusStatus() const;
        [[nodiscard]] Result<bool> IsBc12DetectionComplete() const;
        [[nodiscard]] Result<IcoStatus> GetIcoStatus() const;
        [[nodiscard]] Result<bool> IsInThermalRegulation() const;
        [[nodiscard]] Result<bool> IsDpDmDetectionOngoing() const;
        [[nodiscard]] Result<bool> IsBatteryPresent() const;

        [[nodiscard]] Result<Fault0> GetFault0() const;
        [[nodiscard]] Result<Fault1> GetFault1() const;

        [[nodiscard]] Result<MilliAmperes> GetIbusCurrent() const;
        [[nodiscard]] Result<MilliAmperes> GetIbatCurrent() const;
        [[nodiscard]] Result<MilliVolts> GetVbusVoltage() const;
        [[nodiscard]] Result<MilliVolts> GetVbatVoltage() const;
        [[nodiscard]] Result<MilliVolts> GetVsysVoltage() const;
        [[nodiscard]] Result<NormalizedIntFraction<16>> GetTsPercentage() const;
        [[nodiscard]] Result<Celcius> GetDieTemperature() const;
        [[nodiscard]] Result<MilliVolts> GetUsbDataPlusVoltage() const;
        [[nodiscard]] Result<MilliVolts> GetUsbDataMinusVoltage() const;

        [[nodiscard]] Result<bool> IsAutomaticDpDmDetectionEnabled() const;
        [[nodiscard]] Result<void> SetAutomaticDpDmDetectionEnabled(bool value) const;
        [[nodiscard]] Result<void> ForceDpDmDetection() const;

        [[nodiscard]] Result<DpDac> GetDpDac() const;
        [[nodiscard]] Result<void> SetDpDac(DpDac value) const;
        [[nodiscard]] Result<DmDac> GetDmDac() const;
        [[nodiscard]] Result<void> SetDmDac(DmDac value) const;

    private:
        constexpr static size_t MemorySize = 0x49;

        I2C::Api::IDriver& m_Driver;

        template <typename T>
        Result<void> Read(T reg, uint8_t* data, size_t size) const
        {
            return Read(static_cast<uint8_t>(reg), data, size);
        }

        Result<void> Read(uint8_t offset, uint8_t* data, size_t size) const;

        template <typename T>
        Result<void> Write(T reg, uint8_t* data, size_t size) const
        {
            return Write(static_cast<uint8_t>(reg), data, size);
        }

        Result<void> Write(uint8_t offset, uint8_t* data, size_t size) const;

        template <RegOffset Reg>
        auto ReadField(size_t Start, size_t Num) const
            -> Result<RegisterType_t<GetRegisterSize(Reg)>>
        {
            using ReturnType = RegisterType_t<GetRegisterSize(Reg)>;

            std::array<uint8_t, GetRegisterSize(Reg)> regBytes;

            auto readResult = Read(Reg, regBytes.data(), regBytes.size());
            if (!readResult.has_value())
            {
                return std::unexpected(readResult.error());
            }

            return RegUtils::ReadInt<ReturnType, std::endian::big>(regBytes.data(), Start, Num);
        }

        template <RegOffset Reg, typename T>
        Result<void> WriteField(T value, size_t Start, size_t Num) const
        {
            std::array<uint8_t, GetRegisterSize(Reg)> regBytes;
            auto readResult = Read(Reg, regBytes.data(), regBytes.size());
            if (!readResult.has_value())
            {
                return std::unexpected(readResult.error());
            }
            RegUtils::WriteInt<T, std::endian::big>(value, regBytes.data(), Start, Num);
            return Write(Reg, regBytes.data(), regBytes.size());
        }

        template <RegOffset Reg, typename T>
        auto ReadFieldEnum(size_t start, size_t num) const -> Result<T>
        {
            auto field = ReadField<Reg>(start, num);
            if (!field.has_value())
            {
                return std::unexpected(field.error());
            }
            return static_cast<T>(field.value());
        }

        template <RegOffset Reg, typename T>
        Result<void> WriteFieldEnum(T value, size_t start, size_t num) const
        {
            return WriteField<Reg>(static_cast<std::underlying_type_t<T>>(value), start, num);
        }

        template <RegOffset Reg, typename T>
        auto ReadFieldUnit(size_t start, size_t num, T offset, T bitStep) const -> Result<T>
        {
            auto unit = ReadField<Reg>(start, num);
            if (!unit.has_value())
            {
                return std::unexpected(unit.error());
            }
            return T(unit.value()) * bitStep + offset;
        }

        template <RegOffset Reg, typename T>
        Result<void> WriteFieldUnit(size_t start, size_t num, T value, T offset, T bitStep) const
        {
            auto valueReg = (value.Value - offset.Value) / bitStep.Value;
            return WriteField<Reg>(valueReg, start, num);
        }
    };
}
