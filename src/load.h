#ifndef LOAD_H
#define LOAD_H

#include <shared/value.h>
#include <esp_log.h>

LoadMode getLoadMode(ValueType type);
ValueType getValueType(LoadMode mode);
Value * getValue(LoadMode mode, uint32_t value);

class State {
public:
    State()
        : _mode(LoadMode::OFF),
          _setpoint(new Current(0u)),
          _voltage(new Voltage(0u)),
          _current(new Current(0u)),
          _temperature(0),
          _fault_code(0),
          _gate_control_voltage(new GateControlVoltage(0u)),
          _reading_count(0),
          _invalid_voltage_reading_count(0),
          _invalid_current_reading_count(0),
          _update_gate_voltage_count(0)
    {}

    LoadMode getMode() const { return _mode; }
    void setMode(const LoadMode mode)
    {
        if (mode == LoadMode::OFF)
        {
            setSetpoint(new Current(0u));
        }
        _mode = mode;
    }

    Value* getSetpoint() const { return _setpoint; }
    void setSetpoint(Value* setpoint)
    {
        delete _setpoint;
        _setpoint = setpoint;
        _mode = getLoadMode(setpoint->getType());
    }

    Voltage* getVoltage() const { return _voltage; }
    void setVoltage(Voltage* voltage)
    {
        delete _voltage;
        _voltage = voltage;
    }

    Current* getCurrent() const { return _current; }
    void setCurrent(Current* current)
    {
        delete _current;
        _current = current;
    }

    uint16_t getTemperature() const { return _temperature; }
    void setTemperature(const uint16_t temperature) { _temperature = temperature; }

    uint8_t getFaultCode() const { return _fault_code; }
    void setFaultCode(const uint8_t fault_code) { _fault_code = fault_code; }

    GateControlVoltage* getGateControlVoltage() const { return _gate_control_voltage; }
    void setGateControlVoltage(GateControlVoltage* gate_control_voltage)
    {
        delete _gate_control_voltage;
        _gate_control_voltage = gate_control_voltage;
        _update_gate_voltage_count++;
    }

    uint32_t getReadingCount() const { return _reading_count; }

    uint32_t getInvalidVoltageReadingCount() const { return _invalid_voltage_reading_count; }

    uint32_t getInvalidCurrentReadingCount() const { return _invalid_current_reading_count; }

    void incrementReadingCount(const bool valid, ValueType type)
    {
        _reading_count++;

        if (!valid)
        {
            if (type == ValueType::VOLTAGE)
            {
                _invalid_voltage_reading_count++;
            }
            else if (type == ValueType::CURRENT)
            {
                _invalid_current_reading_count++;
            }
        }
    }

    void clearReadingCounts()
    {
        _reading_count = 0;
        _invalid_voltage_reading_count = 0;
        _invalid_current_reading_count = 0;
    }

    uint32_t getUpdateGateVoltageCount() const { return _update_gate_voltage_count; }

    void clearUpdateGateVoltageCount()
    {
        _update_gate_voltage_count = 0;
    }

private:
    LoadMode _mode;
    Value* _setpoint;
    Voltage* _voltage;
    Current* _current;
    uint16_t _temperature;
    uint8_t _fault_code;
    GateControlVoltage* _gate_control_voltage;
    uint32_t _reading_count;
    uint32_t _invalid_voltage_reading_count;
    uint32_t _invalid_current_reading_count;
    uint32_t _update_gate_voltage_count = 0;

};

#endif // LOAD_H