#include "dac.h"
#include <Arduino.h>

/**
 * @file dac.cpp
 * @brief Implementation of DAC interface for MAX5719GSD+
 * 
 * This file contains the implementation of the DAC class for interfacing
 * with the Analog Devices MAX5719GSD+ 20-bit DAC used to generate the gate control voltage.
 */

/**
 * @brief Initialize DAC pins and SPI interface
 * 
 * Configures the GPIO pins for SPI communication and initializes the SPI interface.
 * Sets up the LDAC pin as an output and initializes it to HIGH state.
 */
void DAC::initialize() const
{
    // Initialize SPI interface (no MISO pin needed for DAC)
    _spi->begin(_sclk, -1, _copi, _cs);
    _spi->setHwCs(true);       // Enable hardware chip select
    
    // Configure LDAC pin as output and set to HIGH (inactive)
    pinMode(_ldac, OUTPUT);
    digitalWrite(_ldac, HIGH);
}

/**
 * @brief Write a value to the 20-bit DAC
 * @param value Value to write (0-1048575 for 20-bit precision)
 * 
 * Writes a value to the MAX5719GSD+ 20-bit DAC using SPI communication.
 * The DAC requires 24 bits to be sent but only uses the 20 most significant bits.
 * Triggers the LDAC pin to update the DAC output.
 */
void DAC::write(const uint32_t value) const
{
    // Begin SPI transaction with DAC-specific settings
    _spi->beginTransaction(SPISettings(500000, SPI_MSBFIRST, SPI_MODE0));
    
    // Transfer 24 bits of data to the DAC (20-bit DAC, 4 LSBs ignored)
    _spi->transferBits(value, nullptr, 24);
    
    // End SPI transaction
    _spi->endTransaction();
    
    // Trigger LDAC pin to update DAC output (pulse LOW then HIGH)
    digitalWrite(_ldac, LOW);   // Trigger DAC update
    digitalWrite(_ldac, HIGH);  // Return to inactive state
}
