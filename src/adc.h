#ifndef ADC_H
#define ADC_H

#include <cstdint>
#include <SPI.h>
#include <functional>

enum class RegisterOperation : uint8_t {
    READ = 0x40,
    WRITE = 0x0,
};

enum class Register8Bit : uint8_t {
    STATUS = 0x00, // Same as COMMS
};

enum class Register16Bit : uint8_t {
    ADCMODE = 0x01,
    IFMODE = 0x02,
    GPIOCON = 0x06,
    ID = 0x07,
    CH0 = 0x10,
    CH1 = 0x11,
    CH2 = 0x12,
    CH3 = 0x13,
    SETUPCON0 = 0x20,
    SETUPCON1 = 0x21,
    SETUPCON2 = 0x22,
    SETUPCON3 = 0x23,
    FILTCON0 = 0x28,
    FILTCON1 = 0x29,
    FILTCON2 = 0x2A,
    FILTCON3 = 0x2B,
};

enum class Register24Bit : uint8_t {
    REGCHECK = 0x03,
    DATA = 0x04,
    OFFSET0 = 0x30,
    OFFSET1 = 0x31,
    OFFSET2 = 0x32,
    OFFSET3 = 0x33,
    GAIN0 = 0x38,
    GAIN1 = 0x39,
    GAIN2 = 0x3A,
    GAIN3 = 0x3B,
};

class Register {
        public:
        static constexpr uint8_t       COMMS = 0x00;
        static constexpr Register8Bit  STATUS = Register8Bit::STATUS; // Same as COMMS
        static constexpr Register16Bit ADCMODE = Register16Bit::ADCMODE; 
        static constexpr Register16Bit IFMODE = Register16Bit::IFMODE;
        static constexpr Register24Bit REGCHECK = Register24Bit::REGCHECK;
        static constexpr Register24Bit DATA = Register24Bit::DATA;
        static constexpr Register16Bit GPIOCON = Register16Bit::GPIOCON;
        static constexpr Register16Bit ID = Register16Bit::ID;
        static constexpr Register16Bit CH0 = Register16Bit::CH0;
        static constexpr Register16Bit CH1 = Register16Bit::CH1;
        static constexpr Register16Bit CH2 = Register16Bit::CH2;
        static constexpr Register16Bit CH3 = Register16Bit::CH3;
        static constexpr Register16Bit SETUPCON0 = Register16Bit::SETUPCON0;
        static constexpr Register16Bit SETUPCON1 = Register16Bit::SETUPCON1;
        static constexpr Register16Bit SETUPCON2 = Register16Bit::SETUPCON2;
        static constexpr Register16Bit SETUPCON3 = Register16Bit::SETUPCON3;
        static constexpr Register16Bit FILTCON0 = Register16Bit::FILTCON0;
        static constexpr Register16Bit FILTCON1 = Register16Bit::FILTCON1;
        static constexpr Register16Bit FILTCON2 = Register16Bit::FILTCON2;
        static constexpr Register16Bit FILTCON3 = Register16Bit::FILTCON3;
        static constexpr Register24Bit OFFSET0 = Register24Bit::OFFSET0;
        static constexpr Register24Bit OFFSET1 = Register24Bit::OFFSET1;
        static constexpr Register24Bit OFFSET2 = Register24Bit::OFFSET2;
        static constexpr Register24Bit OFFSET3 = Register24Bit::OFFSET3;
        static constexpr Register24Bit GAIN0 = Register24Bit::GAIN0;
        static constexpr Register24Bit GAIN1 = Register24Bit::GAIN1;
        static constexpr Register24Bit GAIN2 = Register24Bit::GAIN2;
        static constexpr Register24Bit GAIN3 = Register24Bit::GAIN3;
    };

enum class ChannelInput : uint8_t {
        AIN0 = 0b00000,
        AIN1 = 0b00001,
        AIN2 = 0b00010,
        AIN3 = 0b00011,
        AIN4 = 0b00100,
        TEMP_SENSOR_PLUS = 0b10001,
        TEMP_SENSOR_MINUS = 0b10010,
        DIFF_AVDD_AVSS_PLUS = 0b10011,
        DIFF_AVDD_AVSS_MINUS = 0b10100,
        REF_PLUS = 0b10101,
        REF_MINUS = 0b10110,
};

class Channel {
    private:
        uint8_t _number;
        Register16Bit _register;
        Channel(uint8_t number, Register16Bit reg) : _number(number), _register(reg) {}
    public:
        bool operator == (const Channel &other) const { return _number == other._number; }
        static const Channel CH0;
        static const Channel CH1;
        static const Channel CH2;
        static const Channel CH3;
        uint8_t getNumber() const { return _number; }
        Register16Bit getRegister() const { return _register; }
};

class Setup {
    private:
        uint16_t _number;
        Register16Bit _register;
        Register16Bit _filter_register;
        Setup(uint8_t number, Register16Bit reg, Register16Bit filter_reg) : _number(number), _register(reg), _filter_register(filter_reg) {}
    public:
        static const Setup SETUP0;
        static const Setup SETUP1;
        static const Setup SETUP2;
        static const Setup SETUP3;
        explicit operator uint16_t() const { return _number; }
        Register16Bit getRegister() const { return _register; }
        Register16Bit getFilterRegister() const { return _filter_register; }
};

enum class Filter : uint8_t {
    Sinc5Sinc1 = 0b00,
    Sinc3 = 0b11
};

enum class FilterSampleRate : uint8_t {
    SPS_250000 = 0b00000,
    SPS_125000 = 0b00001,
    SPS_62500  = 0b00010,
    SPS_50000  = 0b00011,
    SPS_31250  = 0b00100,
    SPS_25000  = 0b00101,
    SPS_15625  = 0b00110,
    SPS_10000  = 0b00111,
    SPS_5000   = 0b01000,
    SPS_2500   = 0b01001,
    SPS_1000   = 0b01010,
    SPS_500    = 0b01011,
    SPS_397_5  = 0b01100,
    SPS_200    = 0b01101,
    SPS_100    = 0b01110,
    SPS_59_92  = 0b01111,
    SPS_49_96  = 0b10000,
    SPS_20     = 0b10001,
    SPS_16_66  = 0b10010,
    SPS_10     = 0b10011,
    SPS_5      = 0b10100
};

enum class Mode {
    /**
     * Continuous conversion mode continues making readings
     * By default this requires issuing a read to the DATA register
     * This plus continuous read makes it so you can just clock
     * readings out whenever they are ready without having to issue
     * the command each time
     */
    CONTINUOUS = 0b000,
    /**
     * Single conversion mode only makes a reading when requested
     * then goes into standby
     */
    SINGLE_CONVERSION = 0b001,
    /**
     * Lower power mode
     */
    STANDBY = 0b010,
    /**
     * Even lower power
     */
    POWER_DOWN = 0b011,
    INTERNAL_OFFSET_CALIBRATION = 0b100,
    SYSTEM_OFFSET_CALIBRATION = 0b110,
    SYSTEM_GAIN_CALIBRATION = 0b111
};

class Status {
    private:
        uint8_t _data;
    public:
        explicit Status(uint8_t data) : _data(data) {}
        uint8_t getData() const { return _data; }
        /**
         * Is there a reading ready?
         */
        bool isReady() const { return _data & 0x80; }
        /**
         * Is there an error? Overvoltage/Undervoltage
         */
        bool hasADCError() const { return _data & 0x40; }
        /**
         * Is there a CRC error in one of the communications to the ADC?
         */
        bool hasCRCError() const { return _data & 0x20; }
        /**
         * Is there an integrity error in the registers? This is only applicable
         * When registry check is enabled
         */
        bool hasRegisterIntegrityError() const { return _data & 0x10; }
        /**
         * Which channel does the current reading apply to?
         */
        Channel currentConversionChannel() const;
};

enum class SettleDelay {
    DELAY_0_US   = 0b000,
    DELAY_4_US   = 0b001,
    DELAY_16_US  = 0b010,
    DELAY_40_US  = 0b011,
    DELAY_100_US = 0b100,
    DELAY_200_US = 0b101,
    DELAY_500_US = 0b110,
    DELAY_1_MS   = 0b111
};

enum class ChecksumMode : uint8_t {
    CHECKSUM_DISABLED = 0b00,
    XOR_READ_CRC_WRITE = 0b01,
    CRC = 0b10,
};

class Reading {
    private:
        bool _valid;
        Status _status;
        uint32_t _data;
        unsigned long _time;
    public:
        Reading(bool valid, Status status, uint32_t data, unsigned long time) : _valid(valid), _status(status), _data(data), _time(time) {}
        Status getStatus() const { return _status; }
        uint32_t getData() const { return _data; }
        unsigned long getTime() const { return _time; }
        bool isValid() const { return _valid; }
};

class ADC {
private:
    SPIClass * _spi;
    uint8_t _sclk;
    uint8_t _cipo;
    uint8_t _copi;
    uint8_t _cs;
    ChecksumMode _checksum_mode = ChecksumMode::CHECKSUM_DISABLED;
    bool _append_status = false;
    bool _continuous_read = false;

    void _beginTransaction() const;
    uint8_t _readRegisterNoTransaction(Register8Bit reg) const;
    void _writeRegisterNoTransaction(Register8Bit reg, uint8_t data, bool confirm) const;
    uint16_t _readRegisterNoTransaction(Register16Bit reg) const;
    void _writeRegisterNoTransaction(Register16Bit reg, uint16_t data, bool confirm) const;
    uint32_t _readRegisterNoTransaction(Register24Bit reg) const;
    void _writeRegisterNoTransaction(Register24Bit reg, uint32_t data, bool confirm) const;
    void _endTransaction() const;
public:
    ADC(uint8_t bus, uint8_t sclk, uint8_t cipo, uint8_t copi, uint8_t cs) : _spi(new SPIClass(bus)), _sclk(sclk), _cipo(cipo), _copi(copi), _cs(cs) {};

    void initialize() const;
    void reset() const;
    void internalZeroScaleCalibration() const;
    uint8_t readRegister(Register8Bit reg) const;
    void writeRegister(Register8Bit reg, uint8_t data, bool confirm) const;
    uint16_t readRegister(Register16Bit reg) const;
    void writeRegister(Register16Bit reg, uint16_t data, bool confirm) const;
    uint32_t readRegister(Register24Bit reg) const;
    void writeRegister(Register24Bit reg, uint32_t data, bool confirm) const;

    uint16_t getId() const;
    Status getStatus() const;
    void configureMode(Mode mode, bool single_cycle_settling, SettleDelay settle_delay) const;
    void configureInterface(bool io_strength, bool continuous_read, bool append_status, ChecksumMode crc_mode);
    void configureChannel(Channel channel,Setup setup, ChannelInput positive_input, ChannelInput negative_input) const;
    void disableChannel(Channel channel) const;
    void enableChannel(Channel channel) const;
    void configureSetup(Setup setup, Filter filter, FilterSampleRate sample_rate) const;
    void startReading() const;
    void stopReading() const;
    Reading read() const;
};

#endif // ADC_H