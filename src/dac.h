#ifndef DAC_H
#define DAC_H

#include <SPI.h>

class DAC {
    public:
    DAC(const uint8_t bus, const uint8_t sclk, const uint8_t copi, const uint8_t cs, const uint8_t _ldac) : _spi(new SPIClass(bus)), _sclk(sclk), _copi(copi), _cs(cs), _ldac(_ldac) {};
    void initialize() const;
    void write(uint32_t value) const;

    private:
    SPIClass * _spi;
    uint8_t _sclk;
    uint8_t _copi;
    uint8_t _cs;
    uint8_t _ldac;

};

#endif // DAC_H