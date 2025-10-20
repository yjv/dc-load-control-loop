#ifndef ADC_H
#define ADC_H

#include <cstdint>
#include <SPI.h>
#include <functional>

/**
 * @file adc.h
 * @brief ADC (Analog-to-Digital Converter) interface for electronic load control
 * 
 * This file contains the ADC class and related enums for interfacing with
 * the AD7175-2BRUZ 24-bit sigma-delta ADC used for current and voltage sensing.
 * The ADC provides high-precision measurements with built-in filtering and
 * CRC error checking capabilities.
 */

/**
 * @brief Register operation types for ADC communication
 * 
 * Defines the operation type for SPI register access.
 * The MSB (bit 7) indicates read (1) or write (0) operation.
 */
enum class RegisterOperation : uint8_t {
    READ = 0x40,    // Read operation (bit 7 set)
    WRITE = 0x0,    // Write operation (bit 7 clear)
};

/**
 * @brief 8-bit ADC registers
 * 
 * Defines the 8-bit register addresses for the AD7175-2BRUZ ADC.
 */
enum class Register8Bit : uint8_t {
    STATUS = 0x00, // Status register (same as COMMS register)
};

/**
 * @brief 16-bit ADC registers
 * 
 * Defines the 16-bit register addresses for the AD7175-2BRUZ ADC.
 * These registers control ADC operation, channel configuration, and filtering.
 */
enum class Register16Bit : uint8_t {
    ADCMODE = 0x01,      // ADC mode control register
    IFMODE = 0x02,       // Interface mode control register
    GPIOCON = 0x06,      // GPIO configuration register
    ID = 0x07,           // Device ID register
    CH0 = 0x10,          // Channel 0 configuration register
    CH1 = 0x11,          // Channel 1 configuration register
    CH2 = 0x12,          // Channel 2 configuration register
    CH3 = 0x13,          // Channel 3 configuration register
    SETUPCON0 = 0x20,    // Setup configuration register 0
    SETUPCON1 = 0x21,    // Setup configuration register 1
    SETUPCON2 = 0x22,    // Setup configuration register 2
    SETUPCON3 = 0x23,    // Setup configuration register 3
    FILTCON0 = 0x28,     // Filter configuration register 0
    FILTCON1 = 0x29,     // Filter configuration register 1
    FILTCON2 = 0x2A,     // Filter configuration register 2
    FILTCON3 = 0x2B,     // Filter configuration register 3
};

/**
 * @brief 24-bit ADC registers
 * 
 * Defines the 24-bit register addresses for the AD7175-2BRUZ ADC.
 * These registers contain calibration data and conversion results.
 */
enum class Register24Bit : uint8_t {
    REGCHECK = 0x03,     // Register check register
    DATA = 0x04,         // Data register (conversion results)
    OFFSET0 = 0x30,      // Offset calibration register 0
    OFFSET1 = 0x31,      // Offset calibration register 1
    OFFSET2 = 0x32,      // Offset calibration register 2
    OFFSET3 = 0x33,      // Offset calibration register 3
    GAIN0 = 0x38,        // Gain calibration register 0
    GAIN1 = 0x39,        // Gain calibration register 1
    GAIN2 = 0x3A,        // Gain calibration register 2
    GAIN3 = 0x3B,        // Gain calibration register 3
};

/**
 * @brief Register address constants for ADC communication
 * 
 * This class provides convenient access to all ADC register addresses
 * as constexpr values, allowing for compile-time optimization.
 */
class Register {
public:
    // Communication and status registers
    static constexpr uint8_t       COMMS = 0x00;                    // Communication register
    static constexpr Register8Bit  STATUS = Register8Bit::STATUS;   // Status register (same as COMMS)
    
    // Control registers
    static constexpr Register16Bit ADCMODE = Register16Bit::ADCMODE;    // ADC mode control
    static constexpr Register16Bit IFMODE = Register16Bit::IFMODE;      // Interface mode control
    static constexpr Register16Bit GPIOCON = Register16Bit::GPIOCON;    // GPIO configuration
    static constexpr Register16Bit ID = Register16Bit::ID;              // Device ID
    
    // Channel configuration registers
    static constexpr Register16Bit CH0 = Register16Bit::CH0;            // Channel 0 configuration
    static constexpr Register16Bit CH1 = Register16Bit::CH1;            // Channel 1 configuration
    static constexpr Register16Bit CH2 = Register16Bit::CH2;            // Channel 2 configuration
    static constexpr Register16Bit CH3 = Register16Bit::CH3;            // Channel 3 configuration
    
    // Setup configuration registers
    static constexpr Register16Bit SETUPCON0 = Register16Bit::SETUPCON0;  // Setup 0 configuration
    static constexpr Register16Bit SETUPCON1 = Register16Bit::SETUPCON1;  // Setup 1 configuration
    static constexpr Register16Bit SETUPCON2 = Register16Bit::SETUPCON2;  // Setup 2 configuration
    static constexpr Register16Bit SETUPCON3 = Register16Bit::SETUPCON3;  // Setup 3 configuration
    
    // Filter configuration registers
    static constexpr Register16Bit FILTCON0 = Register16Bit::FILTCON0;    // Filter 0 configuration
    static constexpr Register16Bit FILTCON1 = Register16Bit::FILTCON1;    // Filter 1 configuration
    static constexpr Register16Bit FILTCON2 = Register16Bit::FILTCON2;    // Filter 2 configuration
    static constexpr Register16Bit FILTCON3 = Register16Bit::FILTCON3;    // Filter 3 configuration
    
    // Data and calibration registers
    static constexpr Register24Bit REGCHECK = Register24Bit::REGCHECK;    // Register check
    static constexpr Register24Bit DATA = Register24Bit::DATA;            // Conversion data
    
    // Offset calibration registers
    static constexpr Register24Bit OFFSET0 = Register24Bit::OFFSET0;      // Offset calibration 0
    static constexpr Register24Bit OFFSET1 = Register24Bit::OFFSET1;      // Offset calibration 1
    static constexpr Register24Bit OFFSET2 = Register24Bit::OFFSET2;      // Offset calibration 2
    static constexpr Register24Bit OFFSET3 = Register24Bit::OFFSET3;      // Offset calibration 3
    
    // Gain calibration registers
    static constexpr Register24Bit GAIN0 = Register24Bit::GAIN0;          // Gain calibration 0
    static constexpr Register24Bit GAIN1 = Register24Bit::GAIN1;          // Gain calibration 1
    static constexpr Register24Bit GAIN2 = Register24Bit::GAIN2;          // Gain calibration 2
    static constexpr Register24Bit GAIN3 = Register24Bit::GAIN3;          // Gain calibration 3
};

/**
 * @brief ADC channel input selection
 * 
 * Defines the available analog input pins and special measurement modes
 * for the AD7175-2BRUZ ADC. Each channel can be configured with different
 * input combinations for differential measurements.
 */
enum class ChannelInput : uint8_t {
    // Standard analog inputs
    AIN0 = 0b00000,                    // Analog input 0
    AIN1 = 0b00001,                    // Analog input 1
    AIN2 = 0b00010,                    // Analog input 2
    AIN3 = 0b00011,                    // Analog input 3
    AIN4 = 0b00100,                    // Analog input 4
    
    // Special measurement modes
    TEMP_SENSOR_PLUS = 0b10001,        // Internal temperature sensor positive
    TEMP_SENSOR_MINUS = 0b10010,       // Internal temperature sensor negative
    DIFF_AVDD_AVSS_PLUS = 0b10011,     // Differential AVDD/AVSS positive
    DIFF_AVDD_AVSS_MINUS = 0b10100,    // Differential AVDD/AVSS negative
    REF_PLUS = 0b10101,                // Reference voltage positive
    REF_MINUS = 0b10110,               // Reference voltage negative
};

/**
 * @brief ADC channel representation
 * 
 * Represents an ADC channel with its number and associated register.
 * Provides a type-safe way to reference ADC channels and their configuration.
 */
class Channel {
private:
    uint8_t _number;                    // Channel number (0-3)
    Register16Bit _register;            // Associated configuration register
    
    /**
     * @brief Private constructor for channel creation
     * @param number Channel number
     * @param reg Associated register
     */
    Channel(uint8_t number, Register16Bit reg) : _number(number), _register(reg) {}
    
public:
    /**
     * @brief Equality operator for channel comparison
     * @param other Channel to compare with
     * @return true if channels are equal
     */
    bool operator == (const Channel &other) const { return _number == other._number; }
    
    // Static channel instances
    static const Channel CH0;           // Channel 0
    static const Channel CH1;           // Channel 1
    static const Channel CH2;           // Channel 2
    static const Channel CH3;           // Channel 3
    
    /**
     * @brief Get the channel number
     * @return Channel number (0-3)
     */
    uint8_t getNumber() const { return _number; }
    
    /**
     * @brief Get the associated register
     * @return Register for this channel's configuration
     */
    Register16Bit getRegister() const { return _register; }
};

/**
 * @brief ADC setup configuration
 * 
 * Represents an ADC setup configuration that defines how a channel
 * should be configured. Each setup has associated configuration
 * and filter registers.
 */
class Setup {
private:
    uint16_t _number;                   // Setup number (0-3)
    Register16Bit _register;            // Setup configuration register
    Register16Bit _filter_register;     // Filter configuration register
    
    /**
     * @brief Private constructor for setup creation
     * @param number Setup number
     * @param reg Configuration register
     * @param filter_reg Filter register
     */
    Setup(uint8_t number, Register16Bit reg, Register16Bit filter_reg) 
        : _number(number), _register(reg), _filter_register(filter_reg) {}
        
public:
    // Static setup instances
    static const Setup SETUP0;          // Setup 0
    static const Setup SETUP1;          // Setup 1
    static const Setup SETUP2;          // Setup 2
    static const Setup SETUP3;          // Setup 3
    
    /**
     * @brief Convert to uint16_t for register operations
     * @return Setup number as uint16_t
     */
    explicit operator uint16_t() const { return _number; }
    
    /**
     * @brief Get the configuration register
     * @return Register for setup configuration
     */
    Register16Bit getRegister() const { return _register; }
    
    /**
     * @brief Get the filter register
     * @return Register for filter configuration
     */
    Register16Bit getFilterRegister() const { return _filter_register; }
};

/**
 * @brief ADC filter types
 * 
 * Defines the available digital filter types for the AD7175-2BRUZ ADC.
 * Different filters provide different noise reduction and settling times.
 */
enum class Filter : uint8_t {
    Sinc5Sinc1 = 0b00,    // Sinc5 + Sinc1 filter (default, good balance)
    Sinc3 = 0b11          // Sinc3 filter (faster settling, higher noise)
};

/**
 * @brief ADC sample rates
 * 
 * Defines the available sample rates for the AD7175-2BRUZ ADC.
 * Higher sample rates provide faster response but may have higher noise.
 * Lower sample rates provide better noise performance but slower response.
 */
enum class FilterSampleRate : uint8_t {
    SPS_250000 = 0b00000,  // 250,000 samples per second
    SPS_125000 = 0b00001,  // 125,000 samples per second
    SPS_62500  = 0b00010,  // 62,500 samples per second
    SPS_50000  = 0b00011,  // 50,000 samples per second
    SPS_31250  = 0b00100,  // 31,250 samples per second
    SPS_25000  = 0b00101,  // 25,000 samples per second
    SPS_15625  = 0b00110,  // 15,625 samples per second
    SPS_10000  = 0b00111,  // 10,000 samples per second
    SPS_5000   = 0b01000,  // 5,000 samples per second (used in this system)
    SPS_2500   = 0b01001,  // 2,500 samples per second
    SPS_1000   = 0b01010,  // 1,000 samples per second
    SPS_500    = 0b01011,  // 500 samples per second
    SPS_397_5  = 0b01100,  // 397.5 samples per second
    SPS_200    = 0b01101,  // 200 samples per second
    SPS_100    = 0b01110,  // 100 samples per second
    SPS_59_92  = 0b01111,  // 59.92 samples per second
    SPS_49_96  = 0b10000,  // 49.96 samples per second
    SPS_20     = 0b10001,  // 20 samples per second
    SPS_16_66  = 0b10010,  // 16.66 samples per second
    SPS_10     = 0b10011,  // 10 samples per second
    SPS_5      = 0b10100   // 5 samples per second
};

/**
 * @brief ADC operation modes
 * 
 * Defines the different operating modes for the AD7175-2BRUZ ADC.
 * Each mode provides different power consumption and conversion behavior.
 */
enum class Mode {
    /**
     * @brief Continuous conversion mode
     * 
     * Continues making readings continuously. By default this requires
     * issuing a read to the DATA register. When combined with continuous
     * read mode, readings can be clocked out whenever they are ready
     * without having to issue the command each time.
     */
    CONTINUOUS = 0b000,
    
    /**
     * @brief Single conversion mode
     * 
     * Only makes a reading when requested, then goes into standby mode.
     * Useful for power-sensitive applications where readings are needed
     * only occasionally.
     */
    SINGLE_CONVERSION = 0b001,
    
    /**
     * @brief Standby mode
     * 
     * Lower power mode where the ADC is ready but not actively converting.
     * Faster wake-up time compared to power-down mode.
     */
    STANDBY = 0b010,
    
    /**
     * @brief Power-down mode
     * 
     * Even lower power mode where most ADC circuits are shut down.
     * Requires longer wake-up time but provides maximum power savings.
     */
    POWER_DOWN = 0b011,
    
    /**
     * @brief Internal offset calibration
     * 
     * Performs internal offset calibration to correct for internal
     * ADC offset errors.
     */
    INTERNAL_OFFSET_CALIBRATION = 0b100,
    
    /**
     * @brief System offset calibration
     * 
     * Performs system offset calibration including external circuitry
     * to correct for overall system offset errors.
     */
    SYSTEM_OFFSET_CALIBRATION = 0b110,
    
    /**
     * @brief System gain calibration
     * 
     * Performs system gain calibration including external circuitry
     * to correct for overall system gain errors.
     */
    SYSTEM_GAIN_CALIBRATION = 0b111
};

/**
 * @brief ADC status register representation
 * 
 * Represents the ADC status register and provides methods to check
 * various status conditions including data ready, errors, and channel information.
 */
class Status {
private:
    uint8_t _data;                      // Raw status register data
    
public:
    /**
     * @brief Constructor
     * @param data Raw status register data
     */
    explicit Status(uint8_t data) : _data(data) {}
    
    /**
     * @brief Get raw status data
     * @return Raw status register value
     */
    uint8_t getData() const { return _data; }
    
    /**
     * @brief Check if a reading is ready
     * @return true if new conversion data is available
     */
    bool isReady() const { return _data & 0x80; }
    
    /**
     * @brief Check for ADC errors
     * @return true if overvoltage/undervoltage error detected
     */
    bool hasADCError() const { return _data & 0x40; }
    
    /**
     * @brief Check for CRC errors
     * @return true if CRC error detected in communication
     */
    bool hasCRCError() const { return _data & 0x20; }
    
    /**
     * @brief Check for register integrity errors
     * @return true if register integrity error detected
     * 
     * Note: This is only applicable when registry check is enabled
     */
    bool hasRegisterIntegrityError() const { return _data & 0x10; }
    
    /**
     * @brief Get the channel for the current reading
     * @return Channel that the current reading applies to
     */
    Channel currentConversionChannel() const;
};

/**
 * @brief ADC settle delay options
 * 
 * Defines the settle delay time between channel switches.
 * Longer delays provide better settling but slower channel switching.
 */
enum class SettleDelay {
    DELAY_0_US   = 0b000,    // No settle delay
    DELAY_4_US   = 0b001,    // 4 microsecond settle delay
    DELAY_16_US  = 0b010,    // 16 microsecond settle delay
    DELAY_40_US  = 0b011,    // 40 microsecond settle delay
    DELAY_100_US = 0b100,    // 100 microsecond settle delay
    DELAY_200_US = 0b101,    // 200 microsecond settle delay
    DELAY_500_US = 0b110,    // 500 microsecond settle delay
    DELAY_1_MS   = 0b111     // 1 millisecond settle delay
};

/**
 * @brief ADC checksum modes
 * 
 * Defines the error checking modes for ADC communication.
 * XOR checksum is faster but CRC provides better error detection.
 */
enum class ChecksumMode : uint8_t {
    CHECKSUM_DISABLED = 0b00,    // No error checking
    XOR_READ_CRC_WRITE = 0b01,   // XOR for reads, CRC for writes
    CRC = 0b10,                   // CRC for both reads and writes
};

/**
 * @brief ADC reading data structure
 * 
 * Represents a single ADC reading with associated metadata including
 * validity, status, data, and timestamp.
 */
class Reading {
private:
    bool _valid;                        // Whether the reading is valid
    Status _status;                     // ADC status at time of reading
    uint32_t _data;                     // 24-bit ADC conversion data
    unsigned long _time;                // Timestamp of the reading
    
public:
    /**
     * @brief Constructor
     * @param valid Whether the reading is valid
     * @param status ADC status at time of reading
     * @param data 24-bit ADC conversion data
     * @param time Timestamp of the reading
     */
    Reading(bool valid, Status status, uint32_t data, unsigned long time) 
        : _valid(valid), _status(status), _data(data), _time(time) {}
    
    /**
     * @brief Get the ADC status
     * @return Status register value
     */
    Status getStatus() const { return _status; }
    
    /**
     * @brief Get the conversion data
     * @return 24-bit ADC conversion data
     */
    uint32_t getData() const { return _data; }
    
    /**
     * @brief Get the reading timestamp
     * @return Timestamp in microseconds
     */
    unsigned long getTime() const { return _time; }
    
    /**
     * @brief Check if the reading is valid
     * @return true if the reading passed all error checks
     */
    bool isValid() const { return _valid; }
};

/**
 * @brief ADC interface class for AD7175-2BRUZ
 * 
 * This class provides a high-level interface to the AD7175-2BRUZ 24-bit sigma-delta ADC.
 * It handles SPI communication, register access, channel configuration, and data reading.
 * The ADC is used for high-precision current and voltage sensing in the electronic load.
 */
class ADC {
private:
    SPIClass * _spi;                    // SPI interface object
    uint8_t _sclk;                      // Serial clock pin
    uint8_t _cipo;                      // Controller in, peripheral out pin
    uint8_t _copi;                      // Controller out, peripheral in pin
    uint8_t _cs;                        // Chip select pin
    ChecksumMode _checksum_mode = ChecksumMode::CHECKSUM_DISABLED;  // Error checking mode
    bool _append_status = false;        // Whether to append status to data reads
    bool _continuous_read = false;      // Whether continuous read mode is enabled

    // Private helper methods for SPI transaction management
    /**
     * @brief Begin SPI transaction with ADC
     * 
     * Configures SPI settings for communication with the AD7175-2BRUZ ADC.
     * Uses 1MHz clock, MSB first, and SPI mode 3 (CPOL=1, CPHA=1).
     */
    void _beginTransaction() const;

    /**
     * @brief End SPI transaction with ADC
     * 
     * Ends the current SPI transaction and releases the SPI bus.
     */
    void _endTransaction() const;
    
    // Private register access methods (without transaction management)
    /**
     * @brief Read 8-bit register without transaction management
     * @param reg Register to read
     * @return Register value
     * 
     * Reads an 8-bit register directly without managing SPI transactions.
     * Used internally by public methods that handle transaction management.
     */
    uint8_t _readRegisterNoTransaction(Register8Bit reg) const;

    /**
     * @brief Write 8-bit register without transaction management
     * @param reg Register to write
     * @param data Data to write
     * @param confirm Whether to read back and confirm the write
     * 
     * Writes an 8-bit register directly without managing SPI transactions.
     * Used internally by public methods that handle transaction management.
     */
    void _writeRegisterNoTransaction(Register8Bit reg, uint8_t data, bool confirm) const;

    /**
     * @brief Read 16-bit register without transaction management
     * @param reg Register to read
     * @return Register value
     * 
     * Reads a 16-bit register directly without managing SPI transactions.
     * Used internally by public methods that handle transaction management.
     */
    uint16_t _readRegisterNoTransaction(Register16Bit reg) const;

    /**
     * @brief Write 16-bit register without transaction management
     * @param reg Register to write
     * @param data Data to write
     * @param confirm Whether to read back and confirm the write
     * 
     * Writes a 16-bit register directly without managing SPI transactions.
     * Used internally by public methods that handle transaction management.
     */
    void _writeRegisterNoTransaction(Register16Bit reg, uint16_t data, bool confirm) const;

    /**
     * @brief Read 24-bit register without transaction management
     * @param reg Register to read
     * @return Register value
     * 
     * Reads a 24-bit register directly without managing SPI transactions.
     * Used internally by public methods that handle transaction management.
     */
    uint32_t _readRegisterNoTransaction(Register24Bit reg) const;

    /**
     * @brief Write 24-bit register without transaction management
     * @param reg Register to write
     * @param data Data to write
     * @param confirm Whether to read back and confirm the write
     * 
     * Writes a 24-bit register directly without managing SPI transactions.
     * Used internally by public methods that handle transaction management.
     */
    void _writeRegisterNoTransaction(Register24Bit reg, uint32_t data, bool confirm) const;
public:
    /**
     * @brief Constructor
     * @param bus SPI bus number
     * @param sclk Serial clock pin
     * @param cipo Controller in, peripheral out pin
     * @param copi Controller out, peripheral in pin
     * @param cs Chip select pin
     */
    ADC(uint8_t bus, uint8_t sclk, uint8_t cipo, uint8_t copi, uint8_t cs) 
        : _spi(new SPIClass(bus)), _sclk(sclk), _cipo(cipo), _copi(copi), _cs(cs) {};

    // Initialization and configuration methods
    void initialize() const;                    // Initialize ADC pins and SPI
    void reset() const;                         // Reset ADC to default state
    void internalZeroScaleCalibration() const;  // Perform internal offset calibration
    
    // Register access methods
    uint8_t readRegister(Register8Bit reg) const;
    void writeRegister(Register8Bit reg, uint8_t data, bool confirm) const;
    uint16_t readRegister(Register16Bit reg) const;
    void writeRegister(Register16Bit reg, uint16_t data, bool confirm) const;
    uint32_t readRegister(Register24Bit reg) const;
    void writeRegister(Register24Bit reg, uint32_t data, bool confirm) const;

    // Status and identification methods
    uint16_t getId() const;                     // Get ADC device ID
    Status getStatus() const;                   // Get current ADC status
    
    // Configuration methods
    void configureMode(Mode mode, bool single_cycle_settling, SettleDelay settle_delay) const;
    void configureInterface(bool io_strength, bool continuous_read, bool append_status, ChecksumMode crc_mode, bool dout_ready);
    void configureChannel(Channel channel, Setup setup, ChannelInput positive_input, ChannelInput negative_input) const;
    void disableChannel(Channel channel) const;
    void enableChannel(Channel channel) const;
    void configureSetup(Setup setup, Filter filter, FilterSampleRate sample_rate) const;
    
    // Data reading methods
    void startReading() const;                  // Start continuous reading mode
    void stopReading() const;                   // Stop continuous reading mode
    Reading read() const;                       // Read a single conversion result
};

#endif // ADC_H