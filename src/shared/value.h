#ifndef VALUE_H
#define VALUE_H

#include <stdint.h>

class Voltage;
class Current;
class Power;
class Resistance;

enum ValueType: uint8_t {
    CURRENT,
    VOLTAGE,
    POWER,
    RESISTANCE,
    GATE_CONTROL_VOLTAGE
};

enum ValueFormat: uint8_t {
    CODE,
    VALUE
};

class Value {
protected:
    Value(ValueType type, float value);
    Value(ValueType type, uint32_t code);
public:
    float getValue();
    ValueType getType() const
    {
        return _type;
    }
    ValueFormat getFormat() const
    {
        return _format;
    }
    uint32_t getCode();
    float getMaxValue() const;
    float getMinValue() const;
    uint32_t getMaxCode() const;

private:
    ValueType _type;
    float _value;
    bool _value_initialized;
    uint32_t _code;
    bool _code_initialized;
    ValueFormat _format;
};

class Current: public Value {
    public:
    Power operator*(Voltage& voltage);
    explicit Current(const float value) : Value(ValueType::CURRENT, value) {}
    explicit Current(const uint32_t code) : Value(ValueType::CURRENT, code) {}
};

class Voltage: public Value {
    public:
    Power operator*(Current& current);
    Resistance operator/(Current& current);
    explicit Voltage(const float value) : Value(ValueType::VOLTAGE, value) {}
    explicit Voltage(const uint32_t code) : Value(ValueType::VOLTAGE, code) {}
};

class Power: public Value {
    public:
    explicit Power(const float value) : Value(ValueType::POWER, value) {}
    explicit Power(const uint32_t code) : Value(ValueType::POWER, code) {}
    Voltage operator/(Current& current);
    Current operator/(Voltage& voltage);
};

class Resistance: public Value {
    public:
    explicit Resistance(const float value) : Value(ValueType::RESISTANCE, value) {}
    explicit Resistance(const uint32_t code) : Value(ValueType::RESISTANCE, code) {}
};

class GateControlVoltage: public Value {
    public:
    explicit GateControlVoltage(const float value) : Value(ValueType::GATE_CONTROL_VOLTAGE, value) {}
    explicit GateControlVoltage(const uint32_t code) : Value(ValueType::GATE_CONTROL_VOLTAGE, code) {}
};

union SetPoint
{
    Value value;
    Voltage voltage;
    Current current;
    Power power;
    Resistance resistance;
};

enum class LoadMode: uint8_t {
    OFF,
    CONSTANT_CURRENT,
    CONSTANT_VOLTAGE,
    CONSTANT_POWER,
    CONSTANT_RESISTANCE
};

#endif // VALUE_H