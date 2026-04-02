#pragma once

#include <functional>
#include <chrono>
#include <expected>
#include "PiSubmarine/RegUtils.h"
#include "PiSubmarine/Bq25792/Units.h"
#include "PiSubmarine/NormalizedIntFraction.h"
#include "PiSubmarine/I2C/Api/IDriver.h"

namespace PiSubmarine::Bq25792
{
    using WaitFunc = std::function<void(std::chrono::milliseconds)>;

    enum class ProtocolError
    {
        Ok,
        WriteError,
        ReadError
    };

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

    enum class AdcSpeed
    {
        Resolution15bits = 0,
        Resolution14bits,
        Resolution13bits,
        Resolution12bits
    };

    class Device
    {
    public:
        constexpr static uint8_t Address = 0x6B;

        explicit Device(I2C::Api::IDriver& driver);

        /// <summary>
        /// Gets Minimal System Voltage (VSYSMIN) from Memory Buffer.
        /// </summary>
        /// <returns>VSYSMIN in mV.</returns>
        [[nodiscard]] std::expected<MilliVolts, ProtocolError> GetMinimalSystemVoltage() const;

        /// <summary>
        /// Sets minimal system voltage. Range: 2500mV - 16000mV, bit step size: 250mV
        /// </summary>
        /// <param name="valueMv">Voltage in mV</param>
        ProtocolError SetMinimalSystemVoltage(MilliVolts valueMv) const;


        /// <summary>
        /// Gets maxium charge current.
        /// </summary>
        /// <returns>Current in mA</returns>
        [[nodiscard]] std::expected<MilliAmperes, ProtocolError> GetChargeCurrentLimit() const;

        /// <summary>
        /// Sets maximum charge current. Range: 50mA - 5000mA, bit step size: 10mA.
        /// </summary>
        /// <param name="valueMa">Current in mA</param>
        ProtocolError SetChargeCurrentLimit(MilliAmperes valueMa) const;

        ProtocolError SetTsIgnore(bool value);

        [[nodiscard]] std::expected<bool, ProtocolError> GetTsIgnore() const;

        /*
        [[nodiscard]] IbatReg GetOtgMaxCurrent() const;

        void SetOtgMaxCurrent(IbatReg value);

        [[nodiscard]] bool IsSfetPresent() const;

        void SetSfetPresent(bool value);

        [[nodiscard]] bool IsDischargeCurrentSensingEnabled() const;

        void SetDischargeCurrentSensingEnabled(bool value);

        [[nodiscard]] bool IsIlimHizCurrentLimitEnabled() const;

        void SetIlimHizCurrentLimitEnabled(bool value);

        [[nodiscard]] bool IsDischargeOcpEnabled() const;

        void SetDischargeOcpEnabled(bool value);

        void SetWdRst(bool value);

        [[nodiscard]] bool GetWdRst() const;

        void SetWatchdog(Watchdog value);

        [[nodiscard]] Watchdog GetWatchdog() const;

        [[nodiscard]] AdcSpeed GetAdcSampleSpeed() const;

        void SetAdcSampleSpeed(AdcSpeed value);

        [[nodiscard]] bool IsAdcEnabled() const;

        void SetAdcEnabled(bool value);

        [[nodiscard]] ChargerStatus0Flags GetChargerStatus0() const;

        [[nodiscard]] ChargeStatus GetChargeStatus() const;

        [[nodiscard]] VbusStatus GetVbusStatus() const;

        [[nodiscard]] bool IsBc12DetectionComplete() const;

        [[nodiscard]] IcoStatus GetIcoStatus() const;

        [[nodiscard]] bool IsInThermalRegulation() const;

        [[nodiscard]] bool IsDpDmDetectionOngoing() const;

        [[nodiscard]] bool IsBatteryPresent() const;

        [[nodiscard]] MilliAmperes GetIbusCurrent() const;

        [[nodiscard]] MilliAmperes GetIbatCurrent() const;

        [[nodiscard]] MilliVolts GetVbusVoltage() const;

        [[nodiscard]] MilliVolts GetVbatVoltage() const;

        [[nodiscard]] MilliVolts GetVsysVoltage() const;

        [[nodiscard]] NormalizedIntFraction<16> GetTsPercentage() const;

        [[nodiscard]] Celcius GetDieTemperature() const;

        [[nodiscard]] MilliVolts GetUsbDataPlusVoltage() const;

        [[nodiscard]] MilliVolts GetUsbDataMinusVoltage() const;

        [[nodiscard]] bool IsAutomaticDpDmDetectionEnabled() const;

        void SetAutomaticDpDmDetectionEnabled(bool value);

    */
    private:
        constexpr static size_t MemorySize = 0x49;

        I2C::Api::IDriver& m_Driver;

        template <typename T>
        ProtocolError Read(T reg, uint8_t* data, size_t size) const
        {
            return Read(static_cast<uint8_t>(reg), data, size);
        }

        ProtocolError Read(uint8_t offset, uint8_t* data, size_t size) const;

        template <typename T>
        ProtocolError Write(T reg, uint8_t* data, size_t size) const
        {
            return Write(static_cast<uint8_t>(reg), data, size);
        }

        ProtocolError Write(uint8_t offset, uint8_t* data, size_t size) const;

        template <RegOffset Reg>
        auto ReadField(size_t Start, size_t Num) const
            -> std::expected<RegisterType_t<GetRegisterSize(Reg)>, ProtocolError>
        {
            // Define the return type locally for cleaner code
            using ReturnType = RegisterType_t<GetRegisterSize(Reg)>;

            std::array<uint8_t, GetRegisterSize(Reg)> regBytes;

            ProtocolError error = Read(Reg, regBytes.data(), regBytes.size());
            if (error != ProtocolError::Ok)
            {
                return std::unexpected(error);
            }

            // Pass the deduced ReturnType to your utility
            return RegUtils::ReadInt<ReturnType, std::endian::big>(regBytes.data(), Start, Num);
        }

        template <RegOffset Reg, typename T>
        ProtocolError WriteField(T value, size_t Start, size_t Num) const
        {
            std::array<uint8_t, GetRegisterSize(Reg)> regBytes;
            ProtocolError error = Read(Reg, regBytes.data(), regBytes.size());
            if (error != ProtocolError::Ok)
            {
                return error;
            }
            RegUtils::WriteInt<T, std::endian::big>(value, regBytes.data(), Start, Num);
            return Write(Reg, regBytes.data(), regBytes.size());
        }
    };
}
