#ifndef LOAD_H
#define LOAD_H

#include <shared/value.h>
#include <esp_log.h>
#include <control/control_algorithm.h>

#include "../../../../../.platformio/packages/toolchain-riscv32-esp/riscv32-esp-elf/include/c++/8.4.0/memory"

/**
 * @file load.h
 * @brief Electronic load control system
 * 
 * This file contains the Load and State classes that manage the electronic load
 * system. The Load class provides the main interface for controlling the load,
 * while the State class manages the internal state including measurements,
 * setpoints, and statistics.
 */

namespace load
{
    // Utility functions for converting between ValueType and LoadMode
    LoadMode getLoadMode(ValueType type);
    ValueType getValueType(LoadMode mode);
    Value * getValue(LoadMode mode, uint32_t value);

    /**
     * @brief Electronic load state management
     * 
     * This class manages the internal state of the electronic load system,
     * including current measurements, voltage measurements, setpoints,
     * control parameters, and statistics. It provides methods to update
     * and retrieve the current state of the load.
     */
    class State {
    public:
        /**
         * @brief Constructor - initialize state with default values
         * 
         * Initializes the load state with safe default values:
         * - Mode: OFF
         * - Setpoint: 0A current
         * - Measurements: 0V, 0A
         * - Gate control: 0V
         * - Counters: 0
         */
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

        // Mode control methods
        /**
         * @brief Get current load mode
         * @return Current load mode (OFF, CONSTANT_CURRENT, etc.)
         */
        LoadMode getMode() const { return _mode; }
        
        /**
         * @brief Set load mode
         * @param mode New load mode
         * 
         * Sets the load mode. If mode is OFF, automatically sets setpoint to 0A current.
         */
        void setMode(const LoadMode mode)
        {
            if (mode == LoadMode::OFF)
            {
                setSetpoint(new Current(0u));
            }
            _mode = mode;
        }

        // Setpoint control methods
        /**
         * @brief Get current setpoint
         * @return Shared pointer to current setpoint value
         */
        std::shared_ptr<Value> getSetpoint() const { return _setpoint; }
        
        /**
         * @brief Set new setpoint
         * @param setpoint New setpoint value
         * 
         * Sets a new setpoint value and automatically updates the load mode
         * based on the setpoint's value type.
         */
        void setSetpoint(Value* setpoint)
        {
            _setpoint = std::shared_ptr<Value>(setpoint);
            _mode = getLoadMode(setpoint->getType());
        }

        // Measurement access methods
        /**
         * @brief Get current voltage measurement
         * @return Shared pointer to voltage measurement
         */
        std::shared_ptr<Voltage> getVoltage() const { return _voltage; }
        
        /**
         * @brief Set voltage measurement
         * @param voltage New voltage measurement
         */
        void setVoltage(Voltage* voltage)
        {
            _voltage = std::shared_ptr<Voltage>(voltage);
        }

        /**
         * @brief Get current current measurement
         * @return Shared pointer to current measurement
         */
        std::shared_ptr<Current> getCurrent() const { return _current; }
        
        /**
         * @brief Set current measurement
         * @param current New current measurement
         */
        void setCurrent(Current* current)
        {
            _current = std::shared_ptr<Current>(current);
        }

        // System status methods
        /**
         * @brief Get temperature reading
         * @return Temperature in degrees Celsius
         */
        uint16_t getTemperature() const { return _temperature; }
        
        /**
         * @brief Set temperature reading
         * @param temperature Temperature in degrees Celsius
         */
        void setTemperature(const uint16_t temperature) { _temperature = temperature; }

        /**
         * @brief Get fault code
         * @return Current fault code
         */
        uint8_t getFaultCode() const { return _fault_code; }
        
        /**
         * @brief Set fault code
         * @param fault_code New fault code
         */
        void setFaultCode(const uint8_t fault_code) { _fault_code = fault_code; }

        // Control output methods
        /**
         * @brief Get gate control voltage
         * @return Shared pointer to gate control voltage
         */
        std::shared_ptr<GateControlVoltage> getGateControlVoltage() const { return _gate_control_voltage; }
        
        /**
         * @brief Set gate control voltage
         * @param gate_control_voltage New gate control voltage
         * 
         * Sets the gate control voltage and increments the update counter.
         */
        void setGateControlVoltage(GateControlVoltage* gate_control_voltage)
        {
            _gate_control_voltage = std::shared_ptr<GateControlVoltage>(gate_control_voltage);
            _update_gate_voltage_count++;
        }

        // Statistics methods
        /**
         * @brief Get total reading count
         * @return Total number of ADC readings processed
         */
        uint32_t getReadingCount() const { return _reading_count; }

        /**
         * @brief Get invalid voltage reading count
         * @return Number of invalid voltage readings
         */
        uint32_t getInvalidVoltageReadingCount() const { return _invalid_voltage_reading_count; }

        /**
         * @brief Get invalid current reading count
         * @return Number of invalid current readings
         */
        uint32_t getInvalidCurrentReadingCount() const { return _invalid_current_reading_count; }

        /**
         * @brief Increment reading count and track invalid readings
         * @param valid Whether the reading was valid
         * @param type Type of reading (VOLTAGE or CURRENT)
         * 
         * Updates reading statistics, incrementing total count and
         * invalid count for the specific type if the reading was invalid.
         */
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

        /**
         * @brief Clear all reading count statistics
         * 
         * Resets all reading count statistics to zero.
         */
        void clearReadingCounts()
        {
            _reading_count = 0;
            _invalid_voltage_reading_count = 0;
            _invalid_current_reading_count = 0;
        }

        /**
         * @brief Get gate voltage update count
         * @return Number of times gate voltage has been updated
         */
        uint32_t getUpdateGateVoltageCount() const { return _update_gate_voltage_count; }

        /**
         * @brief Clear gate voltage update count
         * 
         * Resets the gate voltage update counter to zero.
         */
        void clearUpdateGateVoltageCount()
        {
            _update_gate_voltage_count = 0;
        }

        /**
         * @brief Clear all counters
         * 
         * Resets all counters (reading counts and gate voltage updates) to zero.
         */
        void clearCounts()
        {
            clearReadingCounts();
            clearUpdateGateVoltageCount();
        }

    private:
        LoadMode _mode;
        std::shared_ptr<Value> _setpoint;
        std::shared_ptr<Voltage> _voltage;
        std::shared_ptr<Current> _current;
        uint16_t _temperature;
        uint8_t _fault_code;
        std::shared_ptr<GateControlVoltage> _gate_control_voltage;
        uint32_t _reading_count;
        uint32_t _invalid_voltage_reading_count;
        uint32_t _invalid_current_reading_count;
        uint32_t _update_gate_voltage_count = 0;

    };

    /**
     * @brief Electronic load control interface
     * 
     * This class provides the main interface for controlling the electronic load.
     * It manages the load state and control algorithm, providing methods to
     * set setpoints, modes, and access the internal state.
     */
    class Load
    {
    public:
        /**
         * @brief Constructor with existing state
         * @param state Pointer to existing state object
         * @param control_algorithm Pointer to control algorithm
         */
        Load(State* state, ControlAlgorithm* control_algorithm)
            : _state(state),
              _control_algorithm(control_algorithm)
        {
        }

        /**
         * @brief Constructor with new state
         * @param control_algorithm Pointer to control algorithm
         * 
         * Creates a new Load object with a new State instance.
         */
        explicit Load(ControlAlgorithm* control_algorithm)
            : _state(new State()),
              _control_algorithm(control_algorithm)
        {
        }

        // State access methods
        /**
         * @brief Get the load state
         * @return Pointer to the load state object
         */
        State* getState() const
        {
            return _state;
        }

        /**
         * @brief Get the control algorithm
         * @return Pointer to the control algorithm
         */
        ControlAlgorithm* getControlAlgorithm() const
        {
            return _control_algorithm;
        }

        // Control methods
        /**
         * @brief Set new setpoint
         * @param setpoint New setpoint value
         * 
         * Sets a new setpoint and updates the control algorithm target.
         */
        void setSetpoint(Value* setpoint) const
        {
            _state->setSetpoint(setpoint);
            _control_algorithm->setTarget(*_state->getSetpoint());
        }

        /**
         * @brief Set load mode
         * @param mode New load mode
         * 
         * Sets the load mode and updates the control algorithm target.
         */
        void setMode(const LoadMode mode) const
        {
            _state->setMode(mode);
            _control_algorithm->setTarget(*_state->getSetpoint());
        }
        
    private:
        State* _state;                      // Load state management
        ControlAlgorithm* _control_algorithm;  // Control algorithm
    };

}

#endif // LOAD_H