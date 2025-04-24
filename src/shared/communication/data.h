//
// Created by Joseph Deray on 4/8/25.
//

#ifndef DATA_H
#define DATA_H

#include <stdint.h>
#include "../value.h"

template<typename D, uint8_t A>
class ReadRegisterMetadata
{

protected:
    virtual ~ReadRegisterMetadata() = default;
public:
    static constexpr uint8_t ADDRESS = A;
};

template<typename D, uint8_t A>
class ReadWriteRegisterMetadata: public ReadRegisterMetadata<D, A>
{
};

class Buffer
{
public:
    explicit Buffer(uint8_t* buffer, const uint8_t size)
        : _buffer(buffer), _index(0), _size(size)
    {
    }

    uint8_t read8bit();
    uint16_t read16bit();
    uint32_t read32bit();
    void write8bit(uint8_t value);
    void write16bit(uint16_t value);
    void write32bit(uint32_t value);

private:
    uint8_t incrementIndex()
    {
        if (_index >= _size)
        {
            return _size - 1;
        }

        return _index++;
    }

    uint8_t * _buffer;
    uint8_t _index;
    uint8_t _size;
};

template<uint8_t S>
class Data
{
protected:
    ~Data() = default;

public:
    static constexpr uint8_t SIZE = S;

    virtual void toNetwork(Buffer buffer) = 0;
    virtual void fromNetwork(Buffer buffer) = 0;
};

class StatusData final: public Data<12> {
public:
    StatusData() : voltage(Voltage((uint32_t)0)), current(Current((uint32_t)0)), load_mode(LoadMode::OFF), temperature((uint32_t)0), fault_code((uint8_t)0)
    {
    }

    void toNetwork(Buffer buffer) override;
    void fromNetwork(Buffer buffer) override;

    Voltage voltage;
    Current current;
    LoadMode load_mode;
    uint16_t temperature;
    uint8_t fault_code;
};

class StatusRegisterMetadata final : public ReadRegisterMetadata<StatusData, 0x01>
{
};

class ModeData final : public Data<5>
{
public:
    ModeData() : load_mode(LoadMode::OFF), set_point((uint32_t)0)
    {
    }

    ModeData(Value* set_point, const LoadMode load_mode)
        : load_mode(load_mode), set_point(set_point->getCode())
    {
    }

    void toNetwork(Buffer buffer) override;
    void fromNetwork(Buffer buffer) override;

    LoadMode load_mode;
    uint32_t set_point;
};

class ModeRegisterMetadata final : public ReadWriteRegisterMetadata<ModeData, 0x02>
{
};

class DebugData final : public Data<20>
{
public:
    DebugData(): gate_control_voltage(GateControlVoltage((uint32_t)0)), invalid_voltage_reading_count(0),
                 invalid_current_reading_count(0), reading_count(0), update_gate_voltage_count(0)
    {
    }

    void toNetwork(Buffer buffer) override;
    void fromNetwork(Buffer buffer) override;

    GateControlVoltage gate_control_voltage;
    uint32_t update_gate_voltage_count;
    uint32_t reading_count;
    uint32_t invalid_voltage_reading_count;
    uint32_t invalid_current_reading_count;
};

class DebugRegisterMetadata final : public ReadRegisterMetadata<DebugData, 0x03>
{
};

class Registers {
public:
    static const StatusRegisterMetadata STATUS;
    static const ModeRegisterMetadata MODE;
    static const DebugRegisterMetadata DEBUG;

};

#endif //DATA_H
