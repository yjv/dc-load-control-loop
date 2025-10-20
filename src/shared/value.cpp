#include "value.h"

#include <HardwareSerial.h>

/**
 * @file value.cpp
 * @brief Implementation of value system for electronic load control
 * 
 * This file contains the implementation of the Value class hierarchy and
 * related functions for converting between raw codes and physical units.
 * It includes calibration constants and mathematical operations for
 * electrical value calculations.
 */

#ifdef ESP_LOGE
#define LOGE ESP_LOGE
#else
#define LOGE(TAG, MESSAGE) Serial.print(TAG); Serial.print(": "); Serial.println(MESSAGE)
#endif

// Maximum code values for different value types
constexpr uint32_t READING_MAX_CODE = 0xffffff;    // 2^24 - 1 (for ADC readings)
constexpr uint32_t WRITING_MAX_CODE = 0xffffff;    // 2^24 - 1 (for DAC outputs)
constexpr uint32_t DERIVED_MAX_CODE = 0xffffffff;  // 2^32 - 1 (for calculated values)

// Current measurement calibration constants
constexpr uint32_t CURRENT_MAX_CODE = READING_MAX_CODE;
constexpr float CURRENT_MAX_VALUE = 20.0f;  // Maximum expected current: 20A
// Reading margin derived from sense resistor amplifier: 1 / (CURRENT_MAX_VALUE * 0.0047 * 50 / 5)
constexpr float CURRENT_READING_MARGIN = 5 / 4.7;
// Maximum current value at full scale (2^24): 1.268184uA to 21.276596A
constexpr float CURRENT_MAX_CODE_VALUE = CURRENT_MAX_VALUE * CURRENT_READING_MARGIN * static_cast<float>(CURRENT_MAX_CODE) / (static_cast<float>(CURRENT_MAX_CODE) + 1.0f);
// Minimum current value at code 1
constexpr float CURRENT_MIN_CODE_VALUE = CURRENT_MAX_CODE_VALUE / static_cast<float>(CURRENT_MAX_CODE);

// Voltage measurement calibration constants
constexpr uint32_t VOLTAGE_MAX_CODE = READING_MAX_CODE;
constexpr float VOLTAGE_MAX_VALUE = 600.0f;  // Maximum expected voltage: 600V
// Reading margin derived from voltage divider: 1 / (VOLTAGE_MAX_VALUE * 15.8e3 / 2e6 / 5)
constexpr float VOLTAGE_READING_MARGIN = 5.0f / 4.8372558838f;
// Alternative margin calculation (commented): 1.03364389234f
// Maximum voltage value at full scale (2^24): 37.72446uV to 632.91144V
constexpr float VOLTAGE_MAX_CODE_VALUE = VOLTAGE_MAX_VALUE * VOLTAGE_READING_MARGIN * static_cast<float>(VOLTAGE_MAX_CODE) / (static_cast<float>(VOLTAGE_MAX_CODE) + 1.0f);
// Minimum voltage value at code 1
constexpr float VOLTAGE_MIN_CODE_VALUE = VOLTAGE_MAX_CODE_VALUE / static_cast<float>(VOLTAGE_MAX_CODE);

// Power calculation calibration constants
constexpr uint32_t POWER_MAX_CODE = DERIVED_MAX_CODE;
// Maximum expected power value (reduced by 8x for better precision)
// We sacrifice range for precision since 430.9184256kW is excessive for the load
// By dividing by 8, we effectively left-shift the codes by 3 bits, giving better
// precision for low-power measurements: max 1.6832751kW, min 391.91804nW
constexpr float POWER_MAX_VALUE = CURRENT_MAX_CODE_VALUE * VOLTAGE_MAX_CODE_VALUE / 8;
// Maximum power value at full scale (2^32): 391.91804nW to 1.6976007kW
constexpr float POWER_MAX_CODE_VALUE = POWER_MAX_VALUE * static_cast<float>(POWER_MAX_CODE) / (static_cast<float>(POWER_MAX_CODE) + 1.0f);
// Minimum power value at code 1
constexpr float POWER_MIN_CODE_VALUE = POWER_MAX_CODE_VALUE / static_cast<float>(POWER_MAX_CODE);

// Resistance calculation calibration constants
constexpr uint32_t RESISTANCE_MAX_CODE = DERIVED_MAX_CODE;
// Maximum expected resistance value (reduced by 256x for practical range)
// Without reduction, max would be 499.069MOhm (voltage max / current min) which is excessive
// and gives min of 116.2mOhm, insufficient for 10mOhm minimum requirement
// By dividing by 256, we effectively left-shift the codes by 8 bits, giving better
// precision for low-resistance measurements: max 1.949MOhm, min 453.90072uOhm
constexpr float RESISTANCE_MAX_VALUE = VOLTAGE_MAX_CODE_VALUE / CURRENT_MIN_CODE_VALUE / 256;
// Maximum resistance value at full scale (2^32): 453.90072uOhm to 1.949MOhm
constexpr float RESISTANCE_MAX_CODE_VALUE = RESISTANCE_MAX_VALUE * static_cast<float>(RESISTANCE_MAX_CODE) / (static_cast<float>(RESISTANCE_MAX_CODE) + 1.0f);
// Minimum resistance value at code 1
constexpr float RESISTANCE_MIN_CODE_VALUE = RESISTANCE_MAX_CODE_VALUE / static_cast<float>(RESISTANCE_MAX_CODE);

// Gate control voltage calibration constants
constexpr uint32_t GATE_CONTROL_VOLTAGE_MAX_CODE = WRITING_MAX_CODE;
constexpr float GATE_CONTROL_VOLTAGE_MAX_VALUE = 5.0f;  // Maximum gate control voltage: 5V
// Maximum gate control voltage at full scale (2^24): 2.98023224uV to 5.0V
constexpr float GATE_CONTROL_VOLTAGE_MAX_CODE_VALUE = GATE_CONTROL_VOLTAGE_MAX_VALUE * static_cast<float>(GATE_CONTROL_VOLTAGE_MAX_CODE) / (static_cast<float>(GATE_CONTROL_VOLTAGE_MAX_CODE) + 1.0f);
// Minimum gate control voltage at code 1
constexpr float GATE_CONTROL_VOLTAGE_MIN_CODE_VALUE = GATE_CONTROL_VOLTAGE_MAX_CODE_VALUE / static_cast<float>(GATE_CONTROL_VOLTAGE_MAX_CODE);

// Utility template functions
template <typename T>
constexpr T min(T a, T b) { return (a < b) ? a : b; }

template <typename T>
constexpr T max(T a, T b) { return (a > b) ? a : b; }

/**
 * @brief Calculate resistance from current and voltage codes
 * @param current_code Current measurement code
 * @param voltage_code Voltage measurement code
 * @return Resistance code
 * 
 * Calculates resistance code from current and voltage codes.
 * Returns maximum resistance code if current is zero to avoid division by zero.
 */
uint32_t calculateResistance(const uint32_t current_code, const uint32_t voltage_code) {
    if (current_code == 0)
    {
        return RESISTANCE_MAX_CODE;  // Avoid division by zero
    }

    // Left shift 16 bits to account for 8-bit right shift in values and expansion from 24-bit to 32-bit
    const uint64_t temp_resistance_code = (static_cast<uint64_t>(voltage_code) << 16) / max(static_cast<uint64_t>(current_code), static_cast<uint64_t>(1));

    return static_cast<uint32_t>(min(temp_resistance_code, static_cast<uint64_t>(0xffffffff)));
}

/**
 * @brief Calculate resistance from current and voltage values
 * @param current_value Current measurement value
 * @param voltage_value Voltage measurement value
 * @return Resistance value
 * 
 * Calculates resistance value from current and voltage values.
 * Returns maximum resistance value if current is zero to avoid division by zero.
 */
float calculateResistance(const float current_value, const float voltage_value) {

    if (current_value == 0.0f)
    {
        return RESISTANCE_MAX_VALUE;  // Avoid division by zero
    }

    return voltage_value / current_value;
}

/**
 * @brief Calculate power from power code and current/voltage code
 * @param power_code Power code
 * @param current_or_voltage_code Current or voltage code
 * @return Power code
 * 
 * Calculates power code from power code and current or voltage code.
 * Includes bit shifting to account for precision adjustments.
 */
uint32_t calculatePower(const uint32_t power_code, const uint32_t current_or_voltage_code) {
    uint64_t power_code_temp = static_cast<uint64_t>(power_code) * static_cast<uint64_t>(current_or_voltage_code);

    // Left shift 13 bits to account for 3-bit right shift in values and expansion from 24-bit to 32-bit
    power_code_temp = min(power_code_temp, static_cast<uint64_t>(0xffffffff) << 13);
    return static_cast<uint32_t>(power_code_temp >> 13);
}

/**
 * @brief Calculate current or voltage from power code
 * @param power_code Power code
 * @param current_or_voltage_code Current or voltage code
 * @return Current or voltage code
 * 
 * Calculates current or voltage code from power code and the other value.
 * Used for power-based control calculations.
 */
uint32_t calculateCurrentOrVoltageFromPower(const uint32_t power_code, const uint32_t current_or_voltage_code) {
    uint64_t power_code_temp = static_cast<uint64_t>(power_code) << 13;
    return static_cast<uint32_t>(power_code_temp / current_or_voltage_code);
}

/**
 * @brief Calculate power from current and voltage values
 * @param current Current value
 * @param current_or_voltage Current or voltage value
 * @return Power value
 * 
 * Calculates power value from current and voltage values.
 */
float calculatePower(const float current, const float current_or_voltage) {
    return current_or_voltage * current;
}

/**
 * @brief Clamp value between 0 and maximum
 * @param value Value to clamp
 * @param maxValue Maximum allowed value
 * @return Clamped value
 * 
 * Clamps a float value between 0 and the maximum value.
 */
float bracketValue(const float value, const float maxValue) {
    return max(min(value, maxValue), 0.0f);
}

/**
 * @brief Clamp code between 0 and maximum
 * @param code Code to clamp
 * @param max Maximum allowed code
 * @return Clamped code
 * 
 * Clamps a code value between 0 and the maximum code.
 */
uint32_t bracketCode(const uint32_t code, const uint32_t max) {
    return min(code, max);
}

/**
 * @brief Constructor from physical value
 * @param type Type of value
 * @param value Physical value in appropriate units
 * 
 * Creates a Value object from a physical value. The value is clamped
 * to the valid range for the type.
 */
Value::Value(const ValueType type, const float value) :
_type(type),
_value(bracketValue(value, getMaxValue())),
_value_initialized(true),
_code(0), _code_initialized(false),
_format(ValueFormat::VALUE){}

/**
 * @brief Constructor from raw code
 * @param type Type of value
 * @param code Raw digital code
 * 
 * Creates a Value object from a raw code. The code is clamped
 * to the valid range for the type.
 */
Value::Value(const ValueType type, const uint32_t code) :
_type(type),
_value(0),
_value_initialized(false),
_code(bracketCode(code, getMaxCode())),
_code_initialized(true),
_format(ValueFormat::CODE) {}

/**
 * @brief Get the physical value
 * @return Physical value in appropriate units
 * 
 * Converts the raw code to physical units if necessary.
 * Uses lazy evaluation - conversion only happens when needed.
 */
float Value::getValue()
{
    if (!_value_initialized)
    {
        _value = static_cast<float>(_code) * getMinValue();
        _value_initialized = true;
    }

    return _value;
}

/**
 * @brief Get the raw code
 * @return Raw digital code
 * 
 * Converts the physical value to raw code if necessary.
 * Uses lazy evaluation - conversion only happens when needed.
 */
uint32_t Value::getCode()
{
    if (!_code_initialized)
    {
        _code = static_cast<uint32_t>(_value / getMinValue());
        _code_initialized = true;
    }

    return _code;
}

/**
 * @brief Get maximum physical value for this type
 * @return Maximum value in physical units
 * 
 * Returns the maximum physical value for the specific value type.
 * Aborts if an unexpected value type is encountered.
 */
float Value::getMaxValue() const
{
    switch (_type)
    {
    case ValueType::CURRENT:
        return CURRENT_MAX_CODE_VALUE;
    case ValueType::VOLTAGE:
        return VOLTAGE_MAX_CODE_VALUE;
    case ValueType::POWER:
        return POWER_MAX_CODE_VALUE;
    case ValueType::RESISTANCE:
        return RESISTANCE_MAX_CODE_VALUE;
    case ValueType::GATE_CONTROL_VOLTAGE:
        return GATE_CONTROL_VOLTAGE_MAX_CODE_VALUE;
    default:
        LOGE("Value", "Unhandled ValueType in getMaxValue");
        abort(); // Ensure the program fails if an unhandled case is encountered
    }
}

/**
 * @brief Get minimum physical value for this type
 * @return Minimum value in physical units
 * 
 * Returns the minimum physical value for the specific value type.
 * Aborts if an unexpected value type is encountered.
 */
float Value::getMinValue() const
{
    switch (_type)
    {
    case ValueType::CURRENT:
        return CURRENT_MIN_CODE_VALUE;
    case ValueType::VOLTAGE:
        return VOLTAGE_MIN_CODE_VALUE;
    case ValueType::POWER:
        return POWER_MIN_CODE_VALUE;
    case ValueType::RESISTANCE:
        return RESISTANCE_MIN_CODE_VALUE;
    case ValueType::GATE_CONTROL_VOLTAGE:
        return GATE_CONTROL_VOLTAGE_MIN_CODE_VALUE;
    default:
        LOGE("Value", "Unhandled ValueType in getMinValue");
        abort(); // Ensure the program fails if an unhandled case is encountered
    }
}

/**
 * @brief Get maximum raw code for this type
 * @return Maximum raw code value
 * 
 * Returns the maximum raw code for the specific value type.
 * Aborts if an unexpected value type is encountered.
 */
uint32_t Value::getMaxCode() const
{
    switch (_type)
    {
    case ValueType::CURRENT:
        return CURRENT_MAX_CODE;
    case ValueType::VOLTAGE:
        return VOLTAGE_MAX_CODE;
    case ValueType::POWER:
        return POWER_MAX_CODE;
    case ValueType::RESISTANCE:
        return RESISTANCE_MAX_CODE;
    case ValueType::GATE_CONTROL_VOLTAGE:
        return GATE_CONTROL_VOLTAGE_MAX_CODE;
    default:
        LOGE("Value", "Unhandled ValueType in getMaxCode");
        abort(); // Ensure the program fails if an unhandled case is encountered
    }
}

/**
 * @brief Multiply current by voltage to get power
 * @param voltage Voltage to multiply by
 * @return Power result
 * 
 * Calculates power by multiplying current by voltage.
 * Uses the most efficient calculation method based on value formats.
 */
Power Current::operator*(Voltage& voltage)
{
    if (getFormat() == ValueFormat::VALUE && voltage.getFormat() == ValueFormat::VALUE)
    {
        return Power(calculatePower(getValue(), voltage.getValue()));
    }

    return Power(calculatePower(getCode(), voltage.getCode()));
}

/**
 * @brief Multiply voltage by current to get power
 * @param current Current to multiply by
 * @return Power result
 * 
 * Calculates power by multiplying voltage by current.
 * Uses the most efficient calculation method based on value formats.
 */
Power Voltage::operator*(Current& current)
{
    if (getFormat() == ValueFormat::VALUE && current.getFormat() == ValueFormat::VALUE)
    {
        return Power(calculatePower(current.getValue(), getValue()));
    }

    return Power(calculatePower(current.getCode(), getCode()));
}

/**
 * @brief Divide voltage by current to get resistance
 * @param current Current to divide by
 * @return Resistance result
 * 
 * Calculates resistance by dividing voltage by current.
 * Uses the most efficient calculation method based on value formats.
 */
Resistance Voltage::operator/(Current& current)
{
    if (getFormat() == ValueFormat::VALUE && current.getFormat() == ValueFormat::VALUE)
    {
        return Resistance(calculateResistance(current.getValue(), getValue()));
    }

    return Resistance(calculateResistance(current.getCode(), getCode()));
}

/**
 * @brief Divide power by voltage to get current
 * @param voltage Voltage to divide by
 * @return Current result
 * 
 * Calculates current by dividing power by voltage.
 * Uses code-based calculation for efficiency.
 */
Current Power::operator/(Voltage& voltage)
{
    return Current(calculateCurrentOrVoltageFromPower(getCode(), voltage.getCode()));
}

/**
 * @brief Divide power by current to get voltage
 * @param current Current to divide by
 * @return Voltage result
 * 
 * Calculates voltage by dividing power by current.
 * Uses code-based calculation for efficiency.
 */
Voltage Power::operator/(Current& current)
{
    return Voltage(calculateCurrentOrVoltageFromPower(getCode(), current.getCode()));
}

