/**
 * @file data.cpp
 * @brief Data structure implementation for I2C communication
 *
 * This file implements the data structure methods for I2C communication
 * with the electronic load. It includes buffer utilities and serialization
 * methods for converting data to/from network byte order.
 */

#include "data.h"
#include <Arduino.h>

namespace communication
{
    /**
     * @brief Read an 8-bit value from the buffer
     * @return 8-bit value read from the buffer
     *
     * Reads a single byte from the buffer and advances the index.
     * The buffer index is automatically incremented with bounds checking.
     */
    uint8_t Buffer::read8bit()
    {
        return _buffer[incrementIndex()];
    }

    /**
     * @brief Read a 16-bit value from the buffer
     * @return 16-bit value read from the buffer (big-endian)
     *
     * Reads two bytes from the buffer and combines them into a 16-bit value
     * in big-endian format (most significant byte first).
     */
    uint16_t Buffer::read16bit()
    {
        const auto msb = (uint16_t)read8bit();
        const auto lsb = (uint16_t)read8bit();

        return (msb << 8) | lsb;
    }

    /**
     * @brief Read a 32-bit value from the buffer
     * @return 32-bit value read from the buffer (big-endian)
     *
     * Reads four bytes from the buffer and combines them into a 32-bit value
     * in big-endian format (most significant byte first).
     */
    uint32_t Buffer::read32bit()
    {
        const auto msb = (uint32_t)read8bit();
        const auto mid1 = (uint32_t)read8bit();
        const auto mid2 = (uint32_t)read8bit();
        const auto lsb = (uint32_t)read8bit();

        return (msb << 24) | (mid1 << 16) | (mid2 << 8) | lsb;
    }

    /**
     * @brief Write an 8-bit value to the buffer
     * @param value 8-bit value to write
     *
     * Writes a single byte to the buffer and advances the index.
     * The buffer index is automatically incremented with bounds checking.
     */
    void Buffer::write8bit(const uint8_t value)
    {
        _buffer[incrementIndex()] = value;
    }

    /**
     * @brief Write a 16-bit value to the buffer
     * @param value 16-bit value to write (big-endian)
     *
     * Writes a 16-bit value to the buffer in big-endian format
     * (most significant byte first).
     */
    void Buffer::write16bit(const uint16_t value)
    {
        write8bit((uint8_t)(value >> 8));
        write8bit((uint8_t)(value & 0xFF));
    }

    /**
     * @brief Write a 32-bit value to the buffer
     * @param value 32-bit value to write (big-endian)
     *
     * Writes a 32-bit value to the buffer in big-endian format
     * (most significant byte first).
     */
    void Buffer::write32bit(const uint32_t value)
    {
        write8bit((uint8_t)(value >> 24));
        write8bit((uint8_t)((value >> 16) & 0xFF));
        write8bit((uint8_t)((value >> 8) & 0xFF));
        write8bit((uint8_t)(value & 0xFF));
    }

    /**
     * @brief Convert status data from network byte order
     * @param buffer Buffer to read the data from
     *
     * Deserializes the status data from network byte order after I2C reception.
     * The data is read in the order: voltage, current, load_mode, temperature, fault_code.
     */
    void StatusData::fromNetwork(Buffer buffer) {
        this->voltage = Voltage(buffer.read32bit());
        this->current = Current(buffer.read32bit());
        this->load_mode = (LoadMode)buffer.read8bit();
        this->temperature = buffer.read16bit();
        this->fault_code = buffer.read8bit();
    }

    /**
     * @brief Convert status data to network byte order
     * @param buffer Buffer to write the data to
     *
     * Serializes the status data to network byte order for I2C transmission.
     * The data is written in the order: voltage, current, load_mode, temperature, fault_code.
     */
    void StatusData::toNetwork(Buffer buffer) {
        buffer.write32bit(this->voltage.getCode());
        buffer.write32bit(this->current.getCode());
        buffer.write8bit((uint8_t)this->load_mode);
        buffer.write16bit(this->temperature);
        buffer.write8bit(this->fault_code);
    }

    /**
     * @brief Convert mode data from network byte order
     * @param buffer Buffer to read the data from
     *
     * Deserializes the mode data from network byte order after I2C reception.
     * The data is read in the order: load_mode, set_point.
     */
    void ModeData::fromNetwork(Buffer buffer) {
        this->load_mode = (LoadMode)buffer.read8bit();
        this->set_point = buffer.read32bit();
    }

    /**
     * @brief Convert mode data to network byte order
     * @param buffer Buffer to write the data to
     *
     * Serializes the mode data to network byte order for I2C transmission.
     * The data is written in the order: load_mode, set_point.
     */
    void ModeData::toNetwork(Buffer buffer) {
        buffer.write8bit((uint8_t)this->load_mode);
        buffer.write32bit(this->set_point);
    }

    /**
     * @brief Convert debug data from network byte order
     * @param buffer Buffer to read the data from
     *
     * Deserializes the debug data from network byte order after I2C reception.
     * The data is read in the order: gate_control_voltage, update_gate_voltage_count,
     * reading_count, invalid_voltage_reading_count, invalid_current_reading_count.
     */
    void DebugData::fromNetwork(Buffer buffer)
    {
        this->gate_control_voltage = GateControlVoltage(buffer.read32bit());
        this->update_gate_voltage_count = buffer.read32bit();
        this->reading_count = buffer.read32bit();
        this->invalid_voltage_reading_count = buffer.read32bit();
        this->invalid_current_reading_count = buffer.read32bit();
    }

    /**
     * @brief Convert debug data to network byte order
     * @param buffer Buffer to write the data to
     *
     * Serializes the debug data to network byte order for I2C transmission.
     * The data is written in the order: gate_control_voltage, update_gate_voltage_count,
     * reading_count, invalid_voltage_reading_count, invalid_current_reading_count.
     */
    void DebugData::toNetwork(Buffer buffer)
    {
        buffer.write32bit(this->gate_control_voltage.getCode());
        buffer.write32bit(this->update_gate_voltage_count);
        buffer.write32bit(this->reading_count);
        buffer.write32bit(this->invalid_voltage_reading_count);
        buffer.write32bit(this->invalid_current_reading_count);
    }

    // Static register metadata instances
    const StatusRegisterMetadata Registers::STATUS;
    const DebugRegisterMetadata Registers::DEBUG;
    const ModeRegisterMetadata Registers::MODE;
}