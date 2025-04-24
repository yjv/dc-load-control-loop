#include <adc.h>
#include <Arduino.h>
#include <functional>
#include <FastCRC.h>

void ADC::_beginTransaction() const
{
    _spi->beginTransaction(SPISettings(1000000, SPI_MSBFIRST, SPI_MODE2));
}

void ADC::_endTransaction() const
{
    _spi->endTransaction();
}


uint8_t xorChecksum(const uint8_t *data, uint8_t size) {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < size; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

#define AD717X_CRC8_POLYNOMIAL_REPRESENTATION 0x07 /* x8 + x2 + x + 1 */

uint8_t crc8Checksum(const uint8_t * data, uint8_t size)
{
    return FastCRC8().smbus(data, size);
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

void appendChecksum(uint8_t * data, uint8_t size, ChecksumMode mode)
{
    switch (mode)
    {
        case ChecksumMode::CHECKSUM_DISABLED:
            break;
        case ChecksumMode::XOR_READ_CRC_WRITE:
            data[size] = xorChecksum(data, size);
            break;
        case ChecksumMode::CRC:
            data[size] = crc8Checksum(data, size);
            break;
    }
}

bool validateChecksum(const uint8_t * data, uint8_t size, ChecksumMode mode)
{
    uint8_t checksum = 0;

    switch (mode)
    {
        case ChecksumMode::CHECKSUM_DISABLED:
            return true;
        case ChecksumMode::XOR_READ_CRC_WRITE:
            checksum = xorChecksum(data, size - 1);
            break;
        case ChecksumMode::CRC:
            checksum = crc8Checksum(data, size - 1);
            break;
    }

    if (checksum != data[size - 1])
    {
        uint64_t content = 0;

        for (size_t i = 0; i < size - 1; i++)
        {
            content |= data[i] << (8 * (size - 1 - i));
        }

        ESP_DRAM_LOGW("adc", "Register: 0x%02x checksum mismatch: 0x%02x != 0x%02x for content 0x%016x\n", data[0], checksum, data[size - 1], content);
        return false;
    }

    ESP_DRAM_LOGD("adc", "Register: 0x%02x checksum match: 0x%02x == 0x%02x\n", data[0], checksum, data[size - 1]);
    
    return true;
}

uint8_t ADC::_readRegisterNoTransaction(Register8Bit reg) const
{
    uint8_t register_byte = static_cast<uint8_t>(RegisterOperation::READ) | static_cast<uint8_t>(reg);

    uint8_t input[3] = {register_byte, 0, 0};
    uint8_t output[3];

    do
    {
        _spi->transferBytes(input, output, _checksum_mode == ChecksumMode::CHECKSUM_DISABLED ? 2 : 3);
        output[0] = register_byte;
    } while (!validateChecksum(output, 2, _checksum_mode));

    return output[1];
}

void ADC::_writeRegisterNoTransaction(Register8Bit reg, uint8_t data, bool confirm) const
{
    uint8_t register_byte = static_cast<uint8_t>(RegisterOperation::WRITE) | static_cast<uint8_t>(reg);
    uint8_t data_array[] = {register_byte, data, 0};

    appendChecksum(data_array, 2, _checksum_mode);

    uint8_t read_data;

    do
    {
        _spi->writeBytes(data_array, _checksum_mode == ChecksumMode::CHECKSUM_DISABLED ? 2 : 3);
        read_data = _readRegisterNoTransaction(reg);
        ESP_DRAM_LOGI("adc", "Checking register 0x%02x, contents: 0x%04x, data: 0x%04x\n", reg, read_data, data);
    } while (confirm && read_data != data);
}

uint16_t ADC::_readRegisterNoTransaction(Register16Bit reg) const
{
    uint8_t register_byte = static_cast<uint8_t>(RegisterOperation::READ) | static_cast<uint8_t>(reg);

    uint8_t input[4] = {register_byte, 0, 0, 0};
    uint8_t output[4];

    do
    {
        _spi->transferBytes(
            input,
            output,
            _checksum_mode == ChecksumMode::CHECKSUM_DISABLED ? 3 : 4
        );
        output[0] = register_byte;
    } while (!validateChecksum(output, 3, _checksum_mode));

    return (static_cast<uint16_t>(output[1]) << 8) | static_cast<uint16_t>(output[2]);
}

void ADC::_writeRegisterNoTransaction(Register16Bit reg, uint16_t data, bool confirm) const
{
    uint8_t register_byte = static_cast<uint8_t>(RegisterOperation::WRITE) | static_cast<uint8_t>(reg);
    uint8_t data_array[] = {
        register_byte,
        static_cast<uint8_t>((data & 0xff00) >> 8),
        static_cast<uint8_t>(data & 0x00ff),
        0
    };

    appendChecksum(data_array, 3, _checksum_mode);

    uint16_t read_data;

    do
    {
        _spi->writeBytes(data_array, _checksum_mode == ChecksumMode::CHECKSUM_DISABLED ? 3 : 4);
        read_data = _readRegisterNoTransaction(reg);
        ESP_DRAM_LOGI("adc", "Checking register 0x%02x, contents: 0x%04x, data: 0x%04x\n", reg, read_data, data);
    } while (confirm && read_data != data);
}

uint32_t ADC::_readRegisterNoTransaction(Register24Bit reg) const
{
    uint8_t register_byte = static_cast<uint8_t>(RegisterOperation::READ) | static_cast<uint8_t>(reg);

    uint8_t input[5] = {register_byte, 0, 0, 0, 0};
    uint8_t output[5];

    do
    {
        _spi->transferBytes(
            input,
            output,
            _checksum_mode == ChecksumMode::CHECKSUM_DISABLED ? 4 : 5
        );
        output[0] = register_byte;
    } while (!validateChecksum(output, 4, _checksum_mode));

    return (static_cast<uint32_t>(output[1]) << 16) | (static_cast<uint32_t>(output[2]) << 8) | static_cast<uint32_t>(output[3]);
}

void ADC::_writeRegisterNoTransaction(Register24Bit reg, uint32_t data, bool confirm) const
{
    uint8_t register_byte = static_cast<uint8_t>(RegisterOperation::WRITE) | static_cast<uint8_t>(reg);
    uint8_t data_array[] = {
        register_byte,
        static_cast<uint8_t>((data & 0xff0000) >> 16),
        static_cast<uint8_t>((data & 0x00ff00) >> 8),
        static_cast<uint8_t>(data & 0x0000ff),
        0
    };

    appendChecksum(data_array, 4, _checksum_mode);

    uint32_t read_data;

    do
    {
        _spi->writeBytes(data_array, _checksum_mode == ChecksumMode::CHECKSUM_DISABLED ? 4 : 5);
        read_data = _readRegisterNoTransaction(reg);
        ESP_DRAM_LOGI("adc", "Checking register 0x%02x, contents: 0x%04x, data: 0x%04x\n", reg, read_data, data);
    } while (confirm && read_data != data);
}

uint8_t ADC::readRegister(Register8Bit reg) const
{
    _beginTransaction();

    uint8_t data = _readRegisterNoTransaction(reg);

    _endTransaction();

    return data;
}

void ADC::initialize() const
{
    pinMode(_sclk, OUTPUT);
    pinMode(_cipo, INPUT);
    pinMode(_copi, OUTPUT);
    pinMode(_cs, OUTPUT);
    _spi->begin(_sclk, _cipo, _copi, _cs);
    _spi->setHwCs(true);
    _spi->bus();
}

void ADC::reset() const
{
    _beginTransaction();
    uint8_t data[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    _spi->writeBytes(data, 9);
    _endTransaction();
}

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

void ADC::writeRegister(Register8Bit reg, uint8_t data, bool confirm) const
{
    _beginTransaction();

    _writeRegisterNoTransaction(reg, data, confirm);

    _endTransaction();
}

uint16_t ADC::readRegister(Register16Bit reg) const
{
    _beginTransaction();

    uint16_t data = _readRegisterNoTransaction(reg);

    _endTransaction();

    return data;
}

void ADC::writeRegister(Register16Bit reg, uint16_t data, bool confirm) const
{
    _beginTransaction();

    _writeRegisterNoTransaction(reg, data, confirm);

    _endTransaction();
}

uint32_t ADC::readRegister(Register24Bit reg) const
{
    _beginTransaction();

    uint32_t data = _readRegisterNoTransaction(reg);

    _endTransaction();

    return data;
}

void ADC::writeRegister(Register24Bit reg, uint32_t data, bool confirm) const
{
    _beginTransaction();

    _writeRegisterNoTransaction(reg, data, confirm);

    _endTransaction();
}

/**
 * Read from the ID register (mostly useful for testing connection)
 */
uint16_t ADC::getId() const
{
    return readRegister(Register::ID);
}

/**
 * Get the status of the ADC
 */
Status ADC::getStatus() const
{
    uint8_t data = readRegister(Register8Bit::STATUS);

    return Status(data);
}

/**
 * Configure the mode of the ADC
 * single cycle settling makes the ADC not show ready until the internal
 * filter has fully settled.
 * settle delay can be added to account for values that take some time to settle
 * between readings.
 * It appears this only makes a difference when using one channel
 */
void ADC::configureMode(Mode mode, bool single_cycle_settling, SettleDelay settle_delay) const
{
    uint16_t data =  (single_cycle_settling ? 1 : 0) << 13
        | static_cast<uint16_t>(settle_delay) << 8
        | static_cast<uint16_t>(mode) << 4;

    writeRegister(Register::ADCMODE, data, true);
}

/**
 * Configure how we interface with the ADC
 * continuous_read will make the ADC continuously generate readings
 * and can have them read out without issuing a read command to the
 * DATA register
 * append_status will append the contents of the STATUS register to the reading
 * enable_crc will enable CRC checking
 */
void ADC::configureInterface(bool io_strength, bool continuous_read, bool append_status, ChecksumMode checksum_mode)
{
    _append_status = append_status;
    _continuous_read = continuous_read;
    _checksum_mode = checksum_mode;

    uint16_t data = (io_strength ? 1 : 0) << 11
        | (continuous_read ? 1 : 0) << 7
        | (append_status ? 1 : 0) << 6
        | static_cast<uint8_t>(checksum_mode) << 2;

    writeRegister(Register::IFMODE, data, !continuous_read);
}

/**
 * Configure a channel on the ADC. There are 4 channels.
 * Each can have its own set of input pins as well as setup
 * This configures the ADC with the setup we want to use with the given channel as
 * well as which input pin is the negative and which is the positive for the input
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
 * Disable an ADC channel
 */
void ADC::disableChannel(Channel channel) const
{
    _beginTransaction();

    uint16_t data = _readRegisterNoTransaction(channel.getRegister());
    _writeRegisterNoTransaction(channel.getRegister(), data & 0x7fff, true);

    _endTransaction();
}

/**
 * Disable an ADC channel
 */
void ADC::enableChannel(Channel channel) const
{
    _beginTransaction();

    uint16_t data = _readRegisterNoTransaction(channel.getRegister());
    _writeRegisterNoTransaction(channel.getRegister(), data | 0x8000, true);

    _endTransaction();
}

/**
 * Configure one of 4 setups that can be linked to a channel
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

void ADC::startReading() const
{
    _spi->setHwCs(false);
    _beginTransaction();
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, LOW);
}

void ADC::stopReading() const
{
    _endTransaction();
    digitalWrite(_cs, HIGH);
    detachInterrupt(digitalPinToInterrupt(_cipo));
    _spi->setHwCs(true);
}

/**
 * Read an ADC reading. This will decide what data is there based off configurations in the
 * above methods
 */
Reading ADC::read() const
{
    const uint8_t register_byte = static_cast<uint8_t>(RegisterOperation::READ) | static_cast<uint8_t>(Register::DATA);

    uint32_t size = 3;
    uint8_t input[6]{register_byte, 0, 0, 0, 0, 0};
    uint8_t output[6]{0, 0, 0, 0, 0, 0};
    const uint8_t *input_ptr = input;
    uint8_t *output_ptr = output;

    if (_continuous_read)
    {
        input_ptr++;
        output_ptr++;
    }
    else
    {
        size++;
    }

    if (_append_status)
    {
        size++;
    }

    if (_checksum_mode != ChecksumMode::CHECKSUM_DISABLED)
    {
        size++;
    }

    _spi->transferBytes(input_ptr, output_ptr, size);

    if (_continuous_read)
    {
        output_ptr--;
        size++;
    }

    output_ptr[0] = register_byte;

    bool checksum_valid = validateChecksum(output_ptr, size, _checksum_mode);

    uint32_t reading_data = output_ptr[1] << 16 | output_ptr[2] << 8 | output_ptr[3];
    Status status(_append_status ? output_ptr[4] : _readRegisterNoTransaction(Register8Bit::STATUS));

    // ESP_DRAM_LOGD("ADC", "Reading data: 0x%08x, Raw Data: 0x%02x%02x%02x%02x%02x%02x, Size: %d", reading_data, output[0], output[1], output[2], output[3], output[4], output[5], size);

    return {checksum_valid, status, reading_data, micros()};
}

const Channel Channel::CH0(0, Register::CH0);
const Channel Channel::CH1(1, Register::CH1);
const Channel Channel::CH2(2, Register::CH2);
const Channel Channel::CH3(3, Register::CH3);

const Setup Setup::SETUP0(0, Register::SETUPCON0, Register::FILTCON0);
const Setup Setup::SETUP1(1, Register::SETUPCON1, Register::FILTCON1);
const Setup Setup::SETUP2(2, Register::SETUPCON2, Register::FILTCON2);
const Setup Setup::SETUP3(3, Register::SETUPCON3, Register::FILTCON3);

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
