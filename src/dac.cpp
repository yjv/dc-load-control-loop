#include "dac.h"
#include <Arduino.h>

void DAC::initialize() const
{
    _spi->begin(_sclk, -1, _copi, _cs);
    _spi->setHwCs(true);
    pinMode(_ldac, OUTPUT);
    digitalWrite(_ldac, HIGH);
}

void DAC::write(const uint32_t value) const
{
    _spi->beginTransaction(SPISettings(500000, SPI_MSBFIRST, SPI_MODE0));
    _spi->transferBits(value, nullptr, 24);
    _spi->endTransaction();  
    digitalWrite(_ldac, LOW);
    digitalWrite(_ldac, HIGH);
}
