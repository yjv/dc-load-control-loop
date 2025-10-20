#ifndef DAC_H
#define DAC_H

#include <SPI.h>

/**
 * @file dac.h
 * @brief DAC (Digital-to-Analog Converter) interface for electronic load control
 * 
 * This file contains the DAC class for interfacing with the Analog Devices
 * MAX5719GSD+ 20-bit DAC used to generate the gate control voltage for the
 * electronic load. The DAC output controls the MOSFET gate voltage to regulate current flow.
 */

/**
 * @brief DAC interface class for MAX5719GSD+
 * 
 * This class provides a high-level interface to the Analog Devices MAX5719GSD+
 * 20-bit DAC used for generating the gate control voltage. The DAC output is
 * used to control the MOSFET gate voltage in the electronic load circuit.
 */
class DAC {
public:
    /**
     * @brief Constructor
     * @param bus SPI bus number
     * @param sclk Serial clock pin
     * @param copi Controller out, peripheral in pin
     * @param cs Chip select pin
     * @param ldac Load DAC pin (triggers DAC update)
     */
    DAC(const uint8_t bus, const uint8_t sclk, const uint8_t copi, const uint8_t cs, const uint8_t _ldac) 
        : _spi(new SPIClass(bus)), _sclk(sclk), _copi(copi), _cs(cs), _ldac(_ldac) {};
    
    /**
     * @brief Initialize DAC pins and SPI interface
     * 
     * Configures the GPIO pins for SPI communication and initializes the SPI interface.
     * This must be called before any other DAC operations.
     */
    void initialize() const;
    
    /**
     * @brief Write a value to the 20-bit DAC
     * @param value Value to write (0-1048575 for 20-bit precision)
     * 
     * Writes a value to the MAX5719GSD+ 20-bit DAC. The DAC requires 24 bits
     * to be sent via SPI, but only uses the 20 most significant bits.
     * Triggers the LDAC pin to update the output.
     */
    void write(uint32_t value) const;

private:
    SPIClass * _spi;    // SPI interface object
    uint8_t _sclk;      // Serial clock pin
    uint8_t _copi;      // Controller out, peripheral in pin
    uint8_t _cs;        // Chip select pin
    uint8_t _ldac;      // Load DAC pin (triggers DAC update)
};

#endif // DAC_H