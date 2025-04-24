#include "data.h"
#include <Arduino.h>

uint8_t Buffer::read8bit()
{
    return _buffer[incrementIndex()];
}

uint16_t Buffer::read16bit()
{
    const auto msb = (uint16_t)read8bit();
    const auto lsb = (uint16_t)read8bit();

    return (msb << 8) | lsb;
}

uint32_t Buffer::read32bit()
{
    const auto msb = (uint32_t)read8bit();
    const auto mid1 = (uint32_t)read8bit();
    const auto mid2 = (uint32_t)read8bit();
    const auto lsb = (uint32_t)read8bit();

    return (msb << 24) | (mid1 << 16) | (mid2 << 8) | lsb;
}

void Buffer::write8bit(const uint8_t value)
{
    _buffer[incrementIndex()] = value;
}

void Buffer::write16bit(const uint16_t value)
{
    write8bit((uint8_t)(value >> 8));
    write8bit((uint8_t)(value & 0xFF));
}

void Buffer::write32bit(const uint32_t value)
{
    write8bit((uint8_t)(value >> 24));
    write8bit((uint8_t)((value >> 16) & 0xFF));
    write8bit((uint8_t)((value >> 8) & 0xFF));
    write8bit((uint8_t)(value & 0xFF));
}


void StatusData::fromNetwork(Buffer buffer) {
    this->voltage = Voltage(buffer.read32bit());
    this->current = Current(buffer.read32bit());
    this->load_mode = (LoadMode)buffer.read8bit();
    this->temperature = buffer.read16bit();
    this->fault_code = buffer.read8bit();
}

void StatusData::toNetwork(Buffer buffer) {
    buffer.write32bit(this->voltage.getCode());
    buffer.write32bit(this->current.getCode());
    buffer.write8bit((uint8_t)this->load_mode);
    buffer.write16bit(this->temperature);
    buffer.write8bit(this->fault_code);
}

void ModeData::fromNetwork(Buffer buffer) {
    this->load_mode = (LoadMode)buffer.read8bit();
    this->set_point = buffer.read32bit();
}

void ModeData::toNetwork(Buffer buffer) {
    buffer.write8bit((uint8_t)this->load_mode);
    buffer.write32bit(this->set_point);
}

void DebugData::toNetwork(Buffer buffer)
{
    buffer.write32bit(this->gate_control_voltage.getCode());
    buffer.write32bit(this->update_gate_voltage_count);
    buffer.write32bit(this->reading_count);
    buffer.write32bit(this->invalid_voltage_reading_count);
    buffer.write32bit(this->invalid_current_reading_count);
}

void DebugData::fromNetwork(Buffer buffer)
{
    this->gate_control_voltage = GateControlVoltage(buffer.read32bit());
    this->update_gate_voltage_count = buffer.read32bit();
    this->reading_count = buffer.read32bit();
    this->invalid_voltage_reading_count = buffer.read32bit();
    this->invalid_current_reading_count = buffer.read32bit();
}

const StatusRegisterMetadata Registers::STATUS;
const DebugRegisterMetadata Registers::DEBUG;
const ModeRegisterMetadata Registers::MODE;