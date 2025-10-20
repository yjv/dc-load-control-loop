#include <adc.h>
#include <Arduino.h>
#include <functional>
#include <FastCRC.h>

/**
 * @file adc.cpp
 * @brief Implementation of ADC interface for AD7175-2BRUZ
 * 
 * This file contains the implementation of the ADC class for interfacing
 * with the AD7175-2BRUZ 24-bit sigma-delta ADC. It includes SPI communication,
 * register access, error checking, and data reading functionality.
 */

/**
 * @brief Begin SPI transaction with ADC
 * 
 * Configures SPI settings for communication with the AD7175-2BRUZ ADC.
 * Uses 1MHz clock, MSB first, and SPI mode 3 (CPOL=1, CPHA=1).
 */
void ADC::_beginTransaction() const
{
    _spi->beginTransaction(SPISettings(1000000, SPI_MSBFIRST, SPI_MODE3));
}

/**
 * @brief End SPI transaction with ADC
 * 
 * Ends the current SPI transaction and releases the SPI bus.
 */
void ADC::_endTransaction() const
{
    _spi->endTransaction();
}


/**
 * @brief Calculate XOR checksum for data
 * @param data Pointer to data array
 * @param size Number of bytes to checksum
 * @return XOR checksum value
 * 
 * Calculates a simple XOR checksum for error detection.
 * This is used for read operations when XOR checksum mode is enabled.
 */
uint8_t xorChecksum(const uint8_t *data, uint8_t size) {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < size; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

#define AD717X_CRC8_POLYNOMIAL_REPRESENTATION 0x07 /* x8 + x2 + x + 1 */

/**
 * @brief Calculate CRC8 checksum for data
 * @param data Pointer to data array
 * @param size Number of bytes to checksum
 * @return CRC8 checksum value
 * 
 * Calculates a CRC8 checksum using the SMBus polynomial (x8 + x2 + x + 1).
 * This provides better error detection than XOR checksum and is used
 * for write operations and when CRC mode is enabled.
 */
uint8_t crc8Checksum(const uint8_t * data, uint8_t size)
{
    return FastCRC8().smbus(data, size);
    // Alternative manual CRC8 implementation (commented out):
    // uint8_t i   = 0;
    // uint8_t crc = 0;
    //
    // while (size) {
    //     for (i = 0x80; i != 0; i >>= 1) {
    //         if (((crc & 0x80) != 0) != ((*data & i) != 0)) { /* MSB of CRC register XOR input bit from Data */
    //             crc <<= 1;
    //             crc ^= AD717X_CRC8_POLYNOMIAL_REPRESENTATION;
    //         } else {
    //             crc <<= 1;
    //         }
    //     }
    //
    //     // data++;
    //     size--;
    // }
    // return crc;
}

/**
 * @brief Append checksum to data array
 * @param data Pointer to data array (must have space for checksum)
 * @param size Number of data bytes (checksum will be appended at data[size])
 * @param mode Checksum mode to use
 * 
 * Appends the appropriate checksum to the data array based on the checksum mode.
 * The data array must have space for the checksum byte.
 */
void appendChecksum(uint8_t * data, uint8_t size, ChecksumMode mode)
{
    switch (mode)
    {
        case ChecksumMode::CHECKSUM_DISABLED:
            break;  // No checksum to append
        case ChecksumMode::XOR_READ_CRC_WRITE:
            data[size] = xorChecksum(data, size);
            break;
        case ChecksumMode::CRC:
            data[size] = crc8Checksum(data, size);
            break;
    }
}

/**
 * @brief Validate checksum in data array
 * @param data Pointer to data array with checksum
 * @param size Total size including checksum byte
 * @param mode Checksum mode to use for validation
 * @return true if checksum is valid, false otherwise
 * 
 * Validates the checksum in the data array. Logs warnings for checksum mismatches
 * and debug information for successful validations.
 */
bool validateChecksum(const uint8_t * data, uint8_t size, ChecksumMode mode)
{
    uint8_t checksum = 0;

    switch (mode)
    {
        case ChecksumMode::CHECKSUM_DISABLED:
            return true;  // No checksum to validate
        case ChecksumMode::XOR_READ_CRC_WRITE:
            checksum = xorChecksum(data, size - 1);
            break;
        case ChecksumMode::CRC:
            checksum = crc8Checksum(data, size - 1);
            break;
    }

    if (checksum != data[size - 1])
    {
        // Calculate content for debugging
        uint64_t content = 0;
        for (size_t i = 0; i < size - 1; i++)
        {
            content |= data[i] << (8 * (size - 1 - i));
        }

        ESP_LOGW("adc", "Register: 0x%02x checksum mismatch: 0x%02x != 0x%02x for content 0x%016x\n", 
                 data[0], checksum, data[size - 1], content);
        return false;
    }

    ESP_LOGD("adc", "Register: 0x%02x checksum match: 0x%02x == 0x%02x\n", 
                  data[0], checksum, data[size - 1]);
    
    return true;
}

/**
 * @brief Read 8-bit register without transaction management
 * @param reg Register to read
 * @return Register value
 * 
 * Reads an 8-bit register directly without managing SPI transactions.
 * Used internally by public methods that handle transaction management.
 */
uint8_t ADC::_readRegisterNoTransaction(Register8Bit reg) const
{
    // Create register byte with read operation flag
    uint8_t register_byte = static_cast<uint8_t>(RegisterOperation::READ) | static_cast<uint8_t>(reg);

    // Prepare input/output arrays for SPI transfer
    // input[0] = register byte, input[1-2] = dummy bytes for clocking out data
    uint8_t input[3] = {register_byte, 0, 0};
    uint8_t output[3];

    // Retry loop until valid data is received (handles checksum validation)
    do
    {
        // Transfer data: 2 bytes if no checksum, 3 bytes if checksum enabled
        _spi->transferBytes(input, output, _checksum_mode == ChecksumMode::CHECKSUM_DISABLED ? 2 : 3);
        // Restore register byte for checksum validation (SPI may modify it)
        output[0] = register_byte;
    } while (!validateChecksum(output, sizeof(output), _checksum_mode));

    // Return the actual register data (second byte)
    return output[1];
}

/**
 * @brief Write 8-bit register without transaction management
 * @param reg Register to write
 * @param data Data to write
 * @param confirm Whether to read back and confirm the write
 * 
 * Writes an 8-bit register directly without managing SPI transactions.
 * Used internally by public methods that handle transaction management.
 */
void ADC::_writeRegisterNoTransaction(Register8Bit reg, uint8_t data, bool confirm) const
{
    // Create register byte with write operation flag
    uint8_t register_byte = static_cast<uint8_t>(RegisterOperation::WRITE) | static_cast<uint8_t>(reg);
    // Prepare data array: [register_byte, data, checksum_space]
    uint8_t data_array[] = {register_byte, data, 0};

    // Add checksum to the data array if checksum mode is enabled
    appendChecksum(data_array, 2, _checksum_mode);

    uint8_t read_data;

    // Retry loop for write confirmation (if requested)
    do
    {
        // Send write command: 2 bytes if no checksum, 3 bytes if checksum enabled
        _spi->writeBytes(data_array, _checksum_mode == ChecksumMode::CHECKSUM_DISABLED ? 2 : 3);
        // Read back the register to verify the write (if confirmation requested)
        read_data = _readRegisterNoTransaction(reg);
        ESP_DRAM_LOGI("adc", "Checking register 0x%02x, contents: 0x%04x, data: 0x%04x\n", reg, read_data, data);
    } while (confirm && read_data != data);
}

/**
 * @brief Read 16-bit register without transaction management
 * @param reg Register to read
 * @return Register value
 * 
 * Reads a 16-bit register directly without managing SPI transactions.
 * Used internally by public methods that handle transaction management.
 */
uint16_t ADC::_readRegisterNoTransaction(Register16Bit reg) const
{
    // Create register byte with read operation flag
    uint8_t register_byte = static_cast<uint8_t>(RegisterOperation::READ) | static_cast<uint8_t>(reg);

    // Prepare input/output arrays for SPI transfer
    // input[0] = register byte, input[1-3] = dummy bytes for clocking out 16-bit data
    uint8_t input[4] = {register_byte, 0, 0, 0};
    uint8_t output[4];

    // Retry loop until valid data is received (handles checksum validation)
    do
    {
        // Transfer data: 3 bytes if no checksum, 4 bytes if checksum enabled
        _spi->transferBytes(
            input,
            output,
            _checksum_mode == ChecksumMode::CHECKSUM_DISABLED ? 3 : 4
        );
        // Restore register byte for checksum validation (SPI may modify it)
        output[0] = register_byte;
    } while (!validateChecksum(output, sizeof(output), _checksum_mode));

    // Reconstruct 16-bit value from two data bytes (MSB first)
    return (static_cast<uint16_t>(output[1]) << 8) | static_cast<uint16_t>(output[2]);
}

/**
 * @brief Write 16-bit register without transaction management
 * @param reg Register to write
 * @param data Data to write
 * @param confirm Whether to read back and confirm the write
 * 
 * Writes a 16-bit register directly without managing SPI transactions.
 * Used internally by public methods that handle transaction management.
 */
void ADC::_writeRegisterNoTransaction(Register16Bit reg, uint16_t data, bool confirm) const
{
    // Create register byte with write operation flag
    const uint8_t register_byte = static_cast<uint8_t>(RegisterOperation::WRITE) | static_cast<uint8_t>(reg);
    // Prepare data array: [register_byte, MSB, LSB, checksum_space]
    uint8_t data_array[] = {
        register_byte,
        static_cast<uint8_t>((data & 0xff00) >> 8),  // MSB of 16-bit data
        static_cast<uint8_t>(data & 0x00ff),         // LSB of 16-bit data
        0                                            // Space for checksum
    };

    // Add checksum to the data array if checksum mode is enabled
    appendChecksum(data_array, 3, _checksum_mode);

    uint16_t read_data = 0;

    // Retry loop for write confirmation (if requested)
    do
    {
        // Send write command: 3 bytes if no checksum, 4 bytes if checksum enabled
        _spi->writeBytes(data_array, _checksum_mode == ChecksumMode::CHECKSUM_DISABLED ? 3 : 4);

        // Read back the register to verify the write (if confirmation requested)
        if (confirm)
        {
            read_data = _readRegisterNoTransaction(reg);
            ESP_DRAM_LOGI("adc", "Checking register 0x%02x, contents: 0x%04x, data: 0x%04x\n", reg, read_data, data);
        }
    } while (confirm && read_data != data);
}

/**
 * @brief Read 24-bit register without transaction management
 * @param reg Register to read
 * @return Register value
 * 
 * Reads a 24-bit register directly without managing SPI transactions.
 * Used internally by public methods that handle transaction management.
 */
uint32_t ADC::_readRegisterNoTransaction(Register24Bit reg) const
{
    // Create register byte with read operation flag
    uint8_t register_byte = static_cast<uint8_t>(RegisterOperation::READ) | static_cast<uint8_t>(reg);

    // Prepare input/output arrays for SPI transfer
    // input[0] = register byte, input[1-4] = dummy bytes for clocking out 24-bit data
    uint8_t input[5] = {register_byte, 0, 0, 0, 0};
    uint8_t output[5];

    // Retry loop until valid data is received (handles checksum validation)
    do
    {
        // Transfer data: 4 bytes if no checksum, 5 bytes if checksum enabled
        _spi->transferBytes(
            input,
            output,
            _checksum_mode == ChecksumMode::CHECKSUM_DISABLED ? 4 : 5
        );
        // Restore register byte for checksum validation (SPI may modify it)
        output[0] = register_byte;
    } while (!validateChecksum(output, sizeof(output), _checksum_mode));

    // Reconstruct 24-bit value from three data bytes (MSB first)
    return (static_cast<uint32_t>(output[1]) << 16) | (static_cast<uint32_t>(output[2]) << 8) | static_cast<uint32_t>(output[3]);
}

/**
 * @brief Write 24-bit register without transaction management
 * @param reg Register to write
 * @param data Data to write
 * @param confirm Whether to read back and confirm the write
 * 
 * Writes a 24-bit register directly without managing SPI transactions.
 * Used internally by public methods that handle transaction management.
 */
void ADC::_writeRegisterNoTransaction(Register24Bit reg, uint32_t data, bool confirm) const
{
    // Create register byte with write operation flag
    uint8_t register_byte = static_cast<uint8_t>(RegisterOperation::WRITE) | static_cast<uint8_t>(reg);
    // Prepare data array: [register_byte, MSB, MID, LSB, checksum_space]
    uint8_t data_array[] = {
        register_byte,
        static_cast<uint8_t>((data & 0xff0000) >> 16),  // MSB of 24-bit data
        static_cast<uint8_t>((data & 0x00ff00) >> 8),   // MID of 24-bit data
        static_cast<uint8_t>(data & 0x0000ff),          // LSB of 24-bit data
        0                                               // Space for checksum
    };

    // Add checksum to the data array if checksum mode is enabled
    appendChecksum(data_array, 4, _checksum_mode);

    uint32_t read_data;

    // Retry loop for write confirmation (if requested)
    do
    {
        // Send write command: 4 bytes if no checksum, 5 bytes if checksum enabled
        _spi->writeBytes(data_array, _checksum_mode == ChecksumMode::CHECKSUM_DISABLED ? 4 : 5);
        // Read back the register to verify the write (if confirmation requested)
        read_data = _readRegisterNoTransaction(reg);
        ESP_DRAM_LOGI("adc", "Checking register 0x%02x, contents: 0x%04x, data: 0x%04x\n", reg, read_data, data);
    } while (confirm && read_data != data);
}

/**
 * @brief Read an 8-bit register from the ADC
 * @param reg Register to read
 * @return Register value
 * 
 * Reads an 8-bit register from the ADC with proper SPI transaction management.
 * This method handles the SPI transaction lifecycle and delegates to the internal
 * read method that doesn't manage transactions.
 */
uint8_t ADC::readRegister(Register8Bit reg) const
{
    _beginTransaction();

    uint8_t data = _readRegisterNoTransaction(reg);

    _endTransaction();

    return data;
}

/**
 * @brief Initialize ADC pins and SPI interface
 * 
 * Configures the GPIO pins for SPI communication and initializes the SPI interface.
 * This must be called before any other ADC operations.
 */
void ADC::initialize() const
{
    // Configure GPIO pins for SPI communication
    pinMode(_sclk, OUTPUT);    // Serial clock output
    pinMode(_cipo, INPUT);     // Data input from ADC
    pinMode(_copi, OUTPUT);    // Data output to ADC
    pinMode(_cs, OUTPUT);      // Chip select output
    
    // Initialize SPI interface
    _spi->begin(_sclk, _cipo, _copi, _cs);
    _spi->setHwCs(true);       // Enable hardware chip select
    _spi->bus();               // Initialize SPI bus
}

/**
 * @brief Reset ADC to default state
 * 
 * Sends a reset sequence to the ADC by writing 9 bytes of 0xFF.
 * This resets all registers to their default values and puts the ADC
 * in a known state.
 */
void ADC::reset() const
{
    _beginTransaction();
    uint8_t data[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    _spi->writeBytes(data, 9);
    _endTransaction();
}

/**
 * @brief Perform internal zero-scale calibration
 * 
 * Performs an internal zero-scale (offset) calibration of the ADC. This method:
 * 1. Saves the current ADC mode
 * 2. Sets the ADC to internal offset calibration mode
 * 3. Waits for the calibration to complete (indicated by CIPO going low)
 * 4. Restores the previous ADC mode
 * 
 * Note: This method may not work correctly in all configurations. The calibration
 * is intended to correct for internal ADC offset errors at zero input.
 * 
 * Warning: In practice, this method has been observed to make measurements
 * more inaccurate in many cases, so use with caution.
 */
void ADC::internalZeroScaleCalibration() const
{
    _beginTransaction();

    uint16_t previousMode = _readRegisterNoTransaction(Register::ADCMODE);

    _writeRegisterNoTransaction(Register::ADCMODE, static_cast<uint16_t>(Mode::INTERNAL_OFFSET_CALIBRATION) << 4, true);

    while (digitalRead(_cipo) == HIGH)
    {
        delay(1);
    }

    _writeRegisterNoTransaction(Register::ADCMODE, previousMode, true);

    _endTransaction();
}

/**
 * @brief Write an 8-bit register to the ADC
 * @param reg Register to write
 * @param data Data to write
 * @param confirm Whether to read back and confirm the write
 * 
 * Writes an 8-bit register to the ADC with proper SPI transaction management.
 * This method handles the SPI transaction lifecycle and delegates to the internal
 * write method that doesn't manage transactions.
 */
void ADC::writeRegister(Register8Bit reg, uint8_t data, bool confirm) const
{
    _beginTransaction();

    _writeRegisterNoTransaction(reg, data, confirm);

    _endTransaction();
}

/**
 * @brief Read a 16-bit register from the ADC
 * @param reg Register to read
 * @return Register value
 * 
 * Reads a 16-bit register from the ADC with proper SPI transaction management.
 * This method handles the SPI transaction lifecycle and delegates to the internal
 * read method that doesn't manage transactions.
 */
uint16_t ADC::readRegister(Register16Bit reg) const
{
    _beginTransaction();

    uint16_t data = _readRegisterNoTransaction(reg);

    _endTransaction();

    return data;
}

/**
 * @brief Write a 16-bit register to the ADC
 * @param reg Register to write
 * @param data Data to write
 * @param confirm Whether to read back and confirm the write
 * 
 * Writes a 16-bit register to the ADC with proper SPI transaction management.
 * This method handles the SPI transaction lifecycle and delegates to the internal
 * write method that doesn't manage transactions.
 */
void ADC::writeRegister(Register16Bit reg, uint16_t data, bool confirm) const
{
    _beginTransaction();

    _writeRegisterNoTransaction(reg, data, confirm);

    _endTransaction();
}

/**
 * @brief Read a 24-bit register from the ADC
 * @param reg Register to read
 * @return Register value
 * 
 * Reads a 24-bit register from the ADC with proper SPI transaction management.
 * This method handles the SPI transaction lifecycle and delegates to the internal
 * read method that doesn't manage transactions.
 */
uint32_t ADC::readRegister(Register24Bit reg) const
{
    _beginTransaction();

    uint32_t data = _readRegisterNoTransaction(reg);

    _endTransaction();

    return data;
}

/**
 * @brief Write a 24-bit register to the ADC
 * @param reg Register to write
 * @param data Data to write
 * @param confirm Whether to read back and confirm the write
 * 
 * Writes a 24-bit register to the ADC with proper SPI transaction management.
 * This method handles the SPI transaction lifecycle and delegates to the internal
 * write method that doesn't manage transactions.
 */
void ADC::writeRegister(Register24Bit reg, uint32_t data, bool confirm) const
{
    _beginTransaction();

    _writeRegisterNoTransaction(reg, data, confirm);

    _endTransaction();
}

/**
 * @brief Read the ADC device ID
 * @return Device ID from the ID register
 * 
 * Reads from the ID register, mostly useful for testing connection.
 * The ID register contains the device identification information.
 */
uint16_t ADC::getId() const
{
    return readRegister(Register::ID);
}

/**
 * @brief Get the current ADC status
 * @return Status object containing ADC status information
 * 
 * Reads the status register and returns a Status object containing
 * the current state of the ADC including ready flags and error conditions.
 */
Status ADC::getStatus() const
{
    uint8_t data = readRegister(Register8Bit::STATUS);

    return Status(data);
}

/**
 * @brief Configure the mode of the ADC
 * @param mode ADC mode to set
 * @param single_cycle_settling Whether to wait for filter to fully settle
 * @param settle_delay Additional delay for settling between readings
 * 
 * Configures the mode of the ADC. Single cycle settling makes the ADC not show
 * ready until the internal filter has fully settled. Settle delay can be added
 * to account for values that take some time to settle between readings.
 * It appears this only makes a difference when using one channel.
 */
void ADC::configureMode(Mode mode, bool single_cycle_settling, SettleDelay settle_delay) const
{
    uint16_t data =  (single_cycle_settling ? 1 : 0) << 13
        | static_cast<uint16_t>(settle_delay) << 8
        | static_cast<uint16_t>(mode) << 4;

    writeRegister(Register::ADCMODE, data, true);
}

/**
 * @brief Configure how we interface with the ADC
 * @param io_strength I/O strength setting
 * @param continuous_read Whether to enable continuous reading mode
 * @param append_status Whether to append STATUS register to readings
 * @param checksum_mode Checksum mode for error detection
 * @param dout_ready Whether to use DOUT/RDY pin
 * 
 * Configures how we interface with the ADC. Continuous read will make the ADC
 * continuously generate readings and can have them read out without issuing a
 * read command to the DATA register. Append status will append the contents of
 * the STATUS register to the reading. Checksum mode enables error checking.
 */
void ADC::configureInterface(bool io_strength, bool continuous_read, bool append_status, ChecksumMode checksum_mode, bool dout_ready)
{
    _append_status = append_status;
    _continuous_read = continuous_read;
    _checksum_mode = checksum_mode;

    uint16_t data = (io_strength ? 1 : 0) << 11
        | (dout_ready ? 1 : 0) << 8
        | (continuous_read ? 1 : 0) << 7
        | (append_status ? 1 : 0) << 6
        | static_cast<uint8_t>(checksum_mode) << 2
    ;

    writeRegister(Register::IFMODE, data, !continuous_read);
}

/**
 * @brief Configure a channel on the ADC
 * @param channel Channel to configure
 * @param setup Setup to use with this channel
 * @param positive_input Positive input pin for the channel
 * @param negative_input Negative input pin for the channel
 * 
 * Configures a channel on the ADC. There are 4 channels available.
 * Each can have its own set of input pins as well as setup.
 * This configures the ADC with the setup we want to use with the given channel
 * as well as which input pin is the negative and which is the positive for the input.
 */
void ADC::configureChannel(Channel channel, Setup setup, ChannelInput positive_input, ChannelInput negative_input) const
{
    uint16_t data = 0x8000
        | static_cast<uint16_t>(setup) << 12
        | static_cast<uint16_t>(positive_input) << 5
        | static_cast<uint16_t>(negative_input);

    writeRegister(channel.getRegister(), data, true);
}

/**
 * @brief Disable an ADC channel
 * @param channel Channel to disable
 * 
 * Disables the specified ADC channel by clearing the enable bit.
 * The channel will no longer participate in conversions.
 */
void ADC::disableChannel(Channel channel) const
{
    _beginTransaction();

    uint16_t data = _readRegisterNoTransaction(channel.getRegister());
    _writeRegisterNoTransaction(channel.getRegister(), data & 0x7fff, true);

    _endTransaction();
}

/**
 * @brief Enable an ADC channel
 * @param channel Channel to enable
 * 
 * Enables the specified ADC channel by setting the enable bit.
 * The channel will participate in conversions once enabled.
 */
void ADC::enableChannel(Channel channel) const
{
    _beginTransaction();

    uint16_t data = _readRegisterNoTransaction(channel.getRegister());
    _writeRegisterNoTransaction(channel.getRegister(), data | 0x8000, true);

    _endTransaction();
}

/**
 * @brief Configure one of 4 setups that can be linked to a channel
 * @param setup Setup to configure
 * @param filter Filter type to use
 * @param sample_rate Sample rate for the filter
 * 
 * Configures one of the 4 available setups with the specified filter
 * and sample rate. These setups can then be linked to channels for
 * different conversion configurations.
 */
void ADC::configureSetup(Setup setup, Filter filter, FilterSampleRate sample_rate) const
{
    _beginTransaction();

    _writeRegisterNoTransaction(setup.getRegister(), 0, true);

    uint16_t data = static_cast<uint16_t>(filter) << 5
     | static_cast<uint16_t>(sample_rate);

    _writeRegisterNoTransaction(setup.getFilterRegister(), data, true);

    _endTransaction();
}

/**
 * @brief Start continuous reading mode
 * 
 * Enables continuous reading mode by disabling hardware chip select,
 * beginning a SPI transaction, and pulling the chip select pin low.
 * This allows for continuous data streaming from the ADC.
 */
void ADC::startReading() const
{
    _spi->setHwCs(false);
    _beginTransaction();
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, LOW);
}

/**
 * @brief Stop continuous reading mode
 * 
 * Disables continuous reading mode by ending the SPI transaction,
 * pulling the chip select pin high, detaching the interrupt,
 * and re-enabling hardware chip select control.
 */
void ADC::stopReading() const
{
    _endTransaction();
    digitalWrite(_cs, HIGH);
    detachInterrupt(digitalPinToInterrupt(_cipo));
    _spi->setHwCs(true);
}

/**
 * @brief Read a single ADC conversion result
 * @return Reading object containing the conversion data and metadata
 * 
 * Reads a single conversion result from the ADC. The method handles different
 * communication modes (continuous read, status append, checksum validation)
 * and returns a Reading object with the conversion data, status, and timestamp.
 */
Reading ADC::read() const
{
    const uint8_t register_byte = static_cast<uint8_t>(RegisterOperation::READ) | static_cast<uint8_t>(Register::DATA);

    // Calculate transfer size based on configuration
    uint32_t size = 3;  // Base size: register byte + 24-bit data
    uint8_t input[6]{register_byte, 0, 0, 0, 0, 0};
    uint8_t output[6]{0, 0, 0, 0, 0, 0};
    const uint8_t *input_ptr = input;
    uint8_t *output_ptr = output;

    // Adjust for continuous read mode (skip register byte)
    if (_continuous_read)
    {
        input_ptr++;
        output_ptr++;
    }
    else
    {
        size++;  // Include register byte
    }

    // Add status byte if enabled
    if (_append_status)
    {
        size++;
    }

    // Add checksum byte if enabled
    if (_checksum_mode != ChecksumMode::CHECKSUM_DISABLED)
    {
        size++;
    }

    // Perform SPI transfer
    _spi->transferBytes(input_ptr, output_ptr, size);

    // Adjust pointers back for continuous read mode
    if (_continuous_read)
    {
        output_ptr--;
        size++;
    }

    // Set register byte for checksum validation
    output_ptr[0] = register_byte;

    // Validate checksum
    bool checksum_valid = validateChecksum(output_ptr, size, _checksum_mode);

    // Extract 24-bit conversion data
    uint32_t reading_data = output_ptr[1] << 16 | output_ptr[2] << 8 | output_ptr[3];
    
    // Get status (either from appended data or separate read)
    Status status(_append_status ? output_ptr[4] : _readRegisterNoTransaction(Register8Bit::STATUS));

    // Debug logging (commented for performance)
    // ESP_LOGD("ADC", "Reading data: 0x%08x, Raw Data: 0x%02x%02x%02x%02x%02x%02x, Size: %d", reading_data, output[0], output[1], output[2], output[3], output[4], output[5], size);

    return {checksum_valid, status, reading_data, micros()};
}

// Static channel instances
const Channel Channel::CH0(0, Register::CH0);
const Channel Channel::CH1(1, Register::CH1);
const Channel Channel::CH2(2, Register::CH2);
const Channel Channel::CH3(3, Register::CH3);

// Static setup instances
const Setup Setup::SETUP0(0, Register::SETUPCON0, Register::FILTCON0);
const Setup Setup::SETUP1(1, Register::SETUPCON1, Register::FILTCON1);
const Setup Setup::SETUP2(2, Register::SETUPCON2, Register::FILTCON2);
const Setup Setup::SETUP3(3, Register::SETUPCON3, Register::FILTCON3);

// Get the channel that was used for the current conversion
Channel Status::currentConversionChannel() const
{
    switch (_data & 0b11)
    {
        case 0:
        return Channel::CH0;
        break;
        case 1:
        return Channel::CH1;
        break;
        case 2:
        return Channel::CH2;
        break;
        case 3:
        return Channel::CH3;
        break;
    }

    return Channel::CH0;
}
