#include <adc.h>
#include <Arduino.h>
#include <functional>

void ADC::_beginTransaction()
{
    _spi->beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE2));
}

void ADC::_endTransaction()
{
    _spi->endTransaction();
}

void IRAM_ATTR ADC::_read_to_callback()
{
    Reading reading = read();

    if (_read_callback)
    {
        _read_callback(reading);
    }
}

uint8_t ADC::_readRegisterNoTransaction(Register8Bit reg)
{
    uint8_t register_byte = static_cast<uint8_t>(RegisterOperation::READ) | static_cast<uint8_t>(reg);

    uint8_t input[2] = {register_byte, 0};
    uint8_t output[2];

    _spi->transferBytes(input, output, 2);

    return output[1];
}

void ADC::_writeRegisterNoTransaction(Register8Bit reg, uint8_t data, bool confirm)
{
    uint8_t register_byte = static_cast<uint8_t>(RegisterOperation::WRITE) | static_cast<uint8_t>(reg);
    uint8_t data_array[] = {register_byte, data};

    uint8_t read_data;

    do
    {
        _spi->writeBytes(data_array, 2);
        read_data = _readRegisterNoTransaction(reg);
        ESP_LOGI("adc", "Checking register 0x%02x, contents: 0x%04x, data: 0x%04x\n", reg, read_data, data);
    } while (confirm && read_data != data);
}

uint16_t ADC::_readRegisterNoTransaction(Register16Bit reg)
{
    uint8_t register_byte = static_cast<uint8_t>(RegisterOperation::READ) | static_cast<uint8_t>(reg);

    uint8_t input[3] = {register_byte, 0, 0};
    uint8_t output[3];

    _spi->transferBytes(
        input,
        output,
        3
    );

    return (static_cast<uint16_t>(output[1]) << 8) | static_cast<uint16_t>(output[2]);
}

void ADC::_writeRegisterNoTransaction(Register16Bit reg, uint16_t data, bool confirm)
{
    uint8_t register_byte = static_cast<uint8_t>(RegisterOperation::WRITE) | static_cast<uint8_t>(reg);
    uint8_t data_array[] = {
        register_byte,
        static_cast<uint8_t>((data & 0xff00) >> 8),
        static_cast<uint8_t>(data & 0x00ff)
    };

    uint16_t read_data;

    do
    {
        _spi->writeBytes(data_array, 3);
        read_data = _readRegisterNoTransaction(reg);
        ESP_LOGI("adc", "Checking register 0x%02x, contents: 0x%04x, data: 0x%04x\n", reg, read_data, data);
    } while (confirm && read_data != data);
}

uint32_t ADC::_readRegisterNoTransaction(Register24Bit reg)
{
    uint8_t register_byte = static_cast<uint8_t>(RegisterOperation::READ) | static_cast<uint8_t>(reg);
    uint8_t input[4] = {register_byte, 0, 0, 0};
    uint8_t output[4];

    _spi->transferBytes(input, output, 4);
    return (static_cast<uint32_t>(output[1]) << 16) | (static_cast<uint32_t>(output[2]) << 8) | static_cast<uint32_t>(output[3]);
}

void ADC::_writeRegisterNoTransaction(Register24Bit reg, uint32_t data, bool confirm)
{
    uint8_t register_byte = static_cast<uint8_t>(RegisterOperation::WRITE) | static_cast<uint8_t>(reg);
    uint8_t data_array[4] = {
        register_byte,
        static_cast<uint8_t>((data & 0xff0000) >> 16),
        static_cast<uint8_t>((data & 0x00ff00) >> 8),
        static_cast<uint8_t>(data & 0x0000ff)
    };
    
    uint32_t read_data;

    do
    {
        _spi->writeBytes(data_array, 4);
        read_data = _readRegisterNoTransaction(reg);
        ESP_LOGI("adc", "Checking register 0x%02x, contents: 0x%04x, data: 0x%04x\n", reg, read_data, data);
    } while (confirm && read_data != data);
}

uint8_t ADC::readRegister(Register8Bit reg)
{
    _beginTransaction();

    uint8_t data = _readRegisterNoTransaction(reg);

    _endTransaction();

    return data;
}

void ADC::initialize()
{
    pinMode(_sclk, OUTPUT);
    pinMode(_cipo, INPUT);
    pinMode(_copi, OUTPUT);
    pinMode(_cs, OUTPUT);
    _spi->begin(_sclk, _cipo, _copi, _cs);
    _spi->setHwCs(true);
    _spi->bus();
}

void ADC::reset()
{
    _beginTransaction();
    // _spi->setHwCs(false);
    // digitalWrite(_spi->pinSS(), HIGH);
    // digitalWrite(_spi->pinSS(), LOW);
    uint8_t data[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    _spi->writeBytes(data, 8);
    // digitalWrite(_spi->pinSS(), HIGH);
    // _spi->setHwCs(true);
    _endTransaction();
}

void ADC::internalZeroScaleCalibration()
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

void ADC::writeRegister(Register8Bit reg, uint8_t data, bool confirm)
{
    _beginTransaction();

    _writeRegisterNoTransaction(reg, data, confirm);

    _endTransaction();
}

uint16_t ADC::readRegister(Register16Bit reg)
{
    _beginTransaction();

    uint16_t data = _readRegisterNoTransaction(reg);

    _endTransaction();

    return data;
}

void ADC::writeRegister(Register16Bit reg, uint16_t data, bool confirm)
{
    _beginTransaction();

    _writeRegisterNoTransaction(reg, data, confirm);

    _endTransaction();
}

uint32_t ADC::readRegister(Register24Bit reg)
{
    _beginTransaction();

    uint32_t data = _readRegisterNoTransaction(reg);

    _endTransaction();

    return data;
}

void ADC::writeRegister(Register24Bit reg, uint32_t data, bool confirm)
{
    _beginTransaction();

    _writeRegisterNoTransaction(reg, data, confirm);

    _endTransaction();
}

/**
 * Read from the ID register (mostly useful for testing connection)
 */
uint16_t ADC::getId()
{
    return readRegister(Register::ID);
}

/**
 * Get the status of the ADC
 */
Status ADC::getStatus()
{
    uint8_t data = readRegister(Register8Bit::STATUS);

    return Status(data);
}

/**
 * Configure the mode of the ADC
 * single cycle settling makes the ADC not show ready until the internal
 * filter has fully settled
 * settle delay can be added to account for values that take some time to settle
 * between readings.
 * It apears this only makes a difference when using one channel
 */
void ADC::configureMode(Mode mode, bool single_cycle_settling, SettleDelay settle_delay)
{
    uint16_t data =  (single_cycle_settling ? 1 : 0) << 13
        | static_cast<uint16_t>(settle_delay) << 8
        | static_cast<uint16_t>(mode) << 4;

    writeRegister(Register::ADCMODE, data, true);
}

/**
 * Configure how we interface with the ADC
 * continuous_read will make the ADC continuously generate readings
 * and can have them read out without issuing a rrad command to the 
 * DATA register
 * append_status will append the contents of the STATUS register to the reading
 * enable_crc will enable CRC checking
 */
void ADC::configureInterface(bool io_strength, bool continuous_read, bool append_status, CrcMode crc_mode)
{
    _append_status = append_status;
    _continuous_read = continuous_read;
    _crc_mode = crc_mode;

    uint16_t data = (io_strength ? 1 : 0) << 11
        | (continuous_read ? 1 : 0) << 7
        | (append_status ? 1 : 0) << 6
        | static_cast<uint8_t>(crc_mode) << 2;

    writeRegister(Register::IFMODE, data, !continuous_read);
}

/**
 * Configure a channel on the ADC. There are 4 channels.
 * Each can have its own set of input pins as well as setup
 * This configures the ADC with the setup we want to use with the given channel as
 * well as which input pin is the negative and which is the positive for the input
 */
void ADC::configureChannel(Channel channel, Setup setup, ChannelInput positive_input, ChannelInput negative_input)
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
void ADC::disableChannel(Channel channel)
{
    _beginTransaction();

    uint16_t data = _readRegisterNoTransaction(channel.getRegister());
    _writeRegisterNoTransaction(channel.getRegister(), data & 0x7fff, true);

    _endTransaction();
}

/**
 * Disable an ADC channel
 */
void ADC::enableChannel(Channel channel)
{
    _beginTransaction();

    uint16_t data = _readRegisterNoTransaction(channel.getRegister());
    _writeRegisterNoTransaction(channel.getRegister(), data | 0x8000, true);

    _endTransaction();
}

/**
 * Configure one of 4 setups that can be linked to a channel
 */
void ADC::configureSetup(Setup setup, Filter filter, FilterSampleRate sample_rate)
{
    _beginTransaction();

    _writeRegisterNoTransaction(setup.getRegister(), 0, true);

    uint16_t data = static_cast<uint16_t>(filter) << 5
     | static_cast<uint16_t>(sample_rate);

    _writeRegisterNoTransaction(setup.getFilterRegister(), data, true);

    _endTransaction();
}

void ADC::startReading()
{
    _spi->setHwCs(false);
    _beginTransaction();
    digitalWrite(_cs, LOW);
}

void ADC::stopReading()
{
    _endTransaction();
    digitalWrite(_cs, HIGH);
    detachInterrupt(digitalPinToInterrupt(_cipo));
    _spi->setHwCs(true);
}

/**z
 * Read an ADC reading. This will decide what data is there based off configurations in the
 * above methods
 */
Reading ADC::read()
{

    uint8_t register_byte = _continuous_read ? 0 : static_cast<uint8_t>(RegisterOperation::READ) | static_cast<uint8_t>(Register::DATA);

    uint32_t size = 3;
    uint8_t input[5] = {register_byte, 0, 0, 0, 0};
    uint8_t output[5];

    if (!_continuous_read)
    {
        size++;
    }

    if (_append_status)
    {
        size++;
    }

    _spi->transferBytes(input, output, size);

    uint8_t start_index = _continuous_read ? 0 : 1;

    uint32_t reading_data = (output[start_index] << 16) | (output[start_index + 1] << 8) | output[start_index + 2];
    Status status(_append_status ? output[start_index + 3] : _readRegisterNoTransaction(Register8Bit::STATUS));

    return Reading(status, reading_data);
}

const Channel Channel::CH0(0 ,Register::CH0);
const Channel Channel::CH1(1, Register::CH1);
const Channel Channel::CH2(2, Register::CH2);
const Channel Channel::CH3(3, Register::CH3);

const Setup Setup::SETUP0(0, Register::SETUPCON0, Register::FILTCON0);
const Setup Setup::SETUP1(1, Register::SETUPCON1, Register::FILTCON1);
const Setup Setup::SETUP2(2, Register::SETUPCON2, Register::FILTCON2);
const Setup Setup::SETUP3(3, Register::SETUPCON3, Register::FILTCON3);

Channel Status::currentConversionChannel()
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
