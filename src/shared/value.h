#ifndef VALUE_H
#define VALUE_H

#include <stdint.h>

/**
 * @file value.h
 * @brief Value system for electronic load control
 * 
 * This file contains the Value class hierarchy and related enums for representing
 * different types of electrical values (current, voltage, power, resistance) in
 * the electronic load system. Values can be represented as either raw codes or
 * converted to physical units.
 */

// Forward declarations for value classes
class Voltage;
class Current;
class Power;
class Resistance;

/**
 * @brief Types of electrical values
 * 
 * Defines the different types of electrical values that can be represented
 * in the electronic load system.
 */
enum ValueType: uint8_t {
    CURRENT,                // Current in amperes
    VOLTAGE,                // Voltage in volts
    POWER,                  // Power in watts
    RESISTANCE,             // Resistance in ohms
    GATE_CONTROL_VOLTAGE    // Gate control voltage in volts
};

/**
 * @brief Value representation format
 * 
 * Defines how a value is represented - either as a raw code or as a physical value.
 */
enum ValueFormat: uint8_t {
    CODE,   // Raw digital code representation
    VALUE   // Physical unit representation
};

/**
 * @brief Base class for electrical values
 * 
 * This is the base class for all electrical values in the system. It provides
 * conversion between raw digital codes and physical units, and manages the
 * value type and format information.
 */
class Value {
protected:
    /**
     * @brief Constructor from physical value
     * @param type Type of value
     * @param value Physical value in appropriate units
     */
    Value(ValueType type, float value);
    
    /**
     * @brief Constructor from raw code
     * @param type Type of value
     * @param code Raw digital code
     */
    Value(ValueType type, uint32_t code);
    
public:
    /**
     * @brief Get the physical value
     * @return Physical value in appropriate units
     * 
     * Converts the raw code to physical units if necessary.
     */
    float getValue();
    
    /**
     * @brief Get the value type
     * @return Type of this value
     */
    ValueType getType() const
    {
        return _type;
    }
    
    /**
     * @brief Get the value format
     * @return Format of this value (CODE or VALUE)
     */
    ValueFormat getFormat() const
    {
        return _format;
    }
    
    /**
     * @brief Get the raw code
     * @return Raw digital code
     * 
     * Converts the physical value to raw code if necessary.
     */
    uint32_t getCode();
    
    /**
     * @brief Get maximum physical value
     * @return Maximum value in physical units
     */
    float getMaxValue() const;
    
    /**
     * @brief Get minimum physical value
     * @return Minimum value in physical units
     */
    float getMinValue() const;
    
    /**
     * @brief Get maximum raw code
     * @return Maximum raw code value
     */
    uint32_t getMaxCode() const;

private:
    ValueType _type;                // Type of value
    float _value;                   // Physical value
    bool _value_initialized;        // Whether physical value is initialized
    uint32_t _code;                 // Raw digital code
    bool _code_initialized;         // Whether raw code is initialized
    ValueFormat _format;            // Current format (CODE or VALUE)
};

/**
 * @brief Current value class
 * 
 * Represents electrical current in amperes. Provides multiplication operator
 * for calculating power when multiplied by voltage.
 */
class Current: public Value {
public:
    /**
     * @brief Multiply current by voltage to get power
     * @param voltage Voltage to multiply by
     * @return Power result
     */
    Power operator*(Voltage& voltage);
    
    /**
     * @brief Constructor from physical value
     * @param value Current in amperes
     */
    explicit Current(const float value) : Value(ValueType::CURRENT, value) {}
    
    /**
     * @brief Constructor from raw code
     * @param code Raw digital code
     */
    explicit Current(const uint32_t code) : Value(ValueType::CURRENT, code) {}
};

/**
 * @brief Voltage value class
 * 
 * Represents electrical voltage in volts. Provides multiplication operator
 * for calculating power when multiplied by current, and division operator
 * for calculating resistance when divided by current.
 */
class Voltage: public Value {
public:
    /**
     * @brief Multiply voltage by current to get power
     * @param current Current to multiply by
     * @return Power result
     */
    Power operator*(Current& current);
    
    /**
     * @brief Divide voltage by current to get resistance
     * @param current Current to divide by
     * @return Resistance result
     */
    Resistance operator/(Current& current);
    
    /**
     * @brief Constructor from physical value
     * @param value Voltage in volts
     */
    explicit Voltage(const float value) : Value(ValueType::VOLTAGE, value) {}
    
    /**
     * @brief Constructor from raw code
     * @param code Raw digital code
     */
    explicit Voltage(const uint32_t code) : Value(ValueType::VOLTAGE, code) {}
};

/**
 * @brief Power value class
 * 
 * Represents electrical power in watts. Provides division operators
 * for calculating current or voltage when divided by the other.
 */
class Power: public Value {
public:
    /**
     * @brief Constructor from physical value
     * @param value Power in watts
     */
    explicit Power(const float value) : Value(ValueType::POWER, value) {}
    
    /**
     * @brief Constructor from raw code
     * @param code Raw digital code
     */
    explicit Power(const uint32_t code) : Value(ValueType::POWER, code) {}
    
    /**
     * @brief Divide power by voltage to get current
     * @param voltage Voltage to divide by
     * @return Current result
     */
    Voltage operator/(Current& current);
    
    /**
     * @brief Divide power by current to get voltage
     * @param current Current to divide by
     * @return Voltage result
     */
    Current operator/(Voltage& voltage);
};

/**
 * @brief Resistance value class
 * 
 * Represents electrical resistance in ohms.
 */
class Resistance: public Value {
public:
    /**
     * @brief Constructor from physical value
     * @param value Resistance in ohms
     */
    explicit Resistance(const float value) : Value(ValueType::RESISTANCE, value) {}
    
    /**
     * @brief Constructor from raw code
     * @param code Raw digital code
     */
    explicit Resistance(const uint32_t code) : Value(ValueType::RESISTANCE, code) {}
};

/**
 * @brief Gate control voltage class
 * 
 * Represents the gate control voltage used to control the MOSFET
 * in the electronic load circuit.
 */
class GateControlVoltage: public Value {
public:
    /**
     * @brief Constructor from physical value
     * @param value Gate control voltage in volts
     */
    explicit GateControlVoltage(const float value) : Value(ValueType::GATE_CONTROL_VOLTAGE, value) {}
    
    /**
     * @brief Constructor from raw code
     * @param code Raw digital code
     */
    explicit GateControlVoltage(const uint32_t code) : Value(ValueType::GATE_CONTROL_VOLTAGE, code) {}
};

/**
 * @brief Electronic load operating modes
 * 
 * Defines the different operating modes for the electronic load.
 * Each mode corresponds to a different control strategy.
 */
enum class LoadMode: uint8_t {
    OFF,                    // Load is turned off (0A current)
    CONSTANT_CURRENT,       // Constant current mode
    CONSTANT_VOLTAGE,       // Constant voltage mode
    CONSTANT_POWER,         // Constant power mode
    CONSTANT_RESISTANCE     // Constant resistance mode
};

#endif // VALUE_H