#include "value.h"

#include <HardwareSerial.h>

#ifdef ESP_LOGE
#define LOGE ESP_LOGE
#else
#define LOGE(TAG, MESSAGE) Serial.print(TAG); Serial.print(": "); Serial.println(MESSAGE)
#endif

// the max code for read values ie current and voltage
constexpr uint32_t READING_MAX_CODE = 0xffffff; // 2^24 - 1
// the max code for values writen ie gate voltage code
constexpr uint32_t WRITING_MAX_CODE = 0xffffff; // 2^24 - 1
// the max code for derived value ie power and resistance
constexpr uint32_t DERIVED_MAX_CODE = 0xffffffff; // 2^32 - 1

constexpr uint32_t CURRENT_MAX_CODE = READING_MAX_CODE;
// maximum expected current value
constexpr float CURRENT_MAX_VALUE = 20.0f; // 20A
// margin for slight over values. This is derived from the combination of the sense resistor amplifier and such `1 / (CURRENT_MAX_VALUE * .0047 * 50 / 5)`
constexpr float CURRENT_READING_MARGIN = 5 / 4.7;
// value at code 2^24 this gives a value from 1.268184uA to 21.276596A
constexpr float CURRENT_MAX_CODE_VALUE = CURRENT_MAX_VALUE * CURRENT_READING_MARGIN * static_cast<float>(CURRENT_MAX_CODE) / (static_cast<float>(CURRENT_MAX_CODE) + 1.0f);
// value at code 1
constexpr float CURRENT_MIN_CODE_VALUE = CURRENT_MAX_CODE_VALUE / static_cast<float>(CURRENT_MAX_CODE);

constexpr uint32_t VOLTAGE_MAX_CODE = READING_MAX_CODE;
// maximum expected voltage value
constexpr float VOLTAGE_MAX_VALUE = 600.0f; // 600V
// // margin for slight over values this is derived from the voltage divider resistors and such `1 / (VOLTAGE_MAX_VALUE * 15.8e3 / 2e6 / 5)`
// constexpr float VOLTAGE_READING_MARGIN = 5 / 4.74;
constexpr float VOLTAGE_READING_MARGIN = 1.1055555556f;
// value at code 2^24 this gives a value from 37.72446uV to 632.91144V
constexpr float VOLTAGE_MAX_CODE_VALUE = VOLTAGE_MAX_VALUE * VOLTAGE_READING_MARGIN * static_cast<float>(VOLTAGE_MAX_CODE) / (static_cast<float>(VOLTAGE_MAX_CODE) + 1.0f);
// value at code 1
constexpr float VOLTAGE_MIN_CODE_VALUE = VOLTAGE_MAX_CODE_VALUE / static_cast<float>(VOLTAGE_MAX_CODE);

constexpr uint32_t POWER_MAX_CODE = DERIVED_MAX_CODE;
// maximum expected power value. Notice the right shift by 3 bits from the max possible from max current and voltage
// We sacrifice range for more precision since 430.9184256kW is insane for the load and the increased precision is more helpful
// for low current and voltage combinations by having a max of 1.6832751kW and min of 391.91804nW
constexpr float POWER_MAX_VALUE = CURRENT_MAX_CODE_VALUE * VOLTAGE_MAX_CODE_VALUE / 8;
 // value at 2^32 this gives a value from 391.91804nW to 1.6976007kW
constexpr float POWER_MAX_CODE_VALUE = POWER_MAX_VALUE * static_cast<float>(POWER_MAX_CODE) / (static_cast<float>(POWER_MAX_CODE) + 1.0f);
constexpr float POWER_MIN_CODE_VALUE = POWER_MAX_CODE_VALUE / static_cast<float>(POWER_MAX_CODE);

constexpr uint32_t RESISTANCE_MAX_CODE = DERIVED_MAX_CODE;
// maximum expected resistance value. Notice the right shift by 8 bits. This is because if we go
// with the expected range with no shifting the max would be when voltage is at max and current is at min
// this would be 499.069MOhm which is insane and gives a minimum value of max / 2^32 = 116.2mOhm. This isn't small enough to
// handle even the requirement of 10mOhm minimum. If we instead shift the max down by 8 bits, this gives a max
// of 1.949MOhm and a min of 453.90072uOhm
constexpr float RESISTANCE_MAX_VALUE = VOLTAGE_MAX_CODE_VALUE / CURRENT_MIN_CODE_VALUE / 256;
constexpr float RESISTANCE_MAX_CODE_VALUE = RESISTANCE_MAX_VALUE * static_cast<float>(RESISTANCE_MAX_CODE) / (static_cast<float>(RESISTANCE_MAX_CODE) + 1.0f); // value at 2^32 this gives a value from 457.76364uOhm to 1.9660799MOhm
constexpr float RESISTANCE_MIN_CODE_VALUE = RESISTANCE_MAX_CODE_VALUE / static_cast<float>(RESISTANCE_MAX_CODE);

constexpr uint32_t GATE_CONTROL_VOLTAGE_MAX_CODE = WRITING_MAX_CODE;
// value at 2^24 this gives a value from 2.98023224uV to 1.280kV
constexpr float GATE_CONTROL_VOLTAGE_MAX_VALUE = 5.0f;
// value at 2^24 this gives a value from 2.98023224uV to 5.0V
constexpr float GATE_CONTROL_VOLTAGE_MAX_CODE_VALUE = GATE_CONTROL_VOLTAGE_MAX_VALUE * static_cast<float>(GATE_CONTROL_VOLTAGE_MAX_CODE) / (static_cast<float>(GATE_CONTROL_VOLTAGE_MAX_CODE) + 1.0f);
constexpr float GATE_CONTROL_VOLTAGE_MIN_CODE_VALUE = GATE_CONTROL_VOLTAGE_MAX_CODE_VALUE / static_cast<float>(GATE_CONTROL_VOLTAGE_MAX_CODE);

template <typename T>
constexpr T min(T a, T b) { return (a < b) ? a : b; }

template <typename T>
constexpr T max(T a, T b) { return (a > b) ? a : b; }

uint32_t calculateResistance(const uint32_t current_code, const uint32_t voltage_code) {
    if (current_code == 0)
    {
        return RESISTANCE_MAX_CODE;
    }

    // this is left shifted 16 bits to account for the 8 bit rightshift in the values and the expansion from
    // 24 bit to 32 bit values
    const uint64_t temp_resistance_code = (static_cast<uint64_t>(voltage_code) << 16) / max(static_cast<uint64_t>(current_code), static_cast<uint64_t>(1));

    return static_cast<uint32_t>(min(temp_resistance_code, static_cast<uint64_t>(0xffffffff)));
}

float calculateResistance(const float current_value, const float voltage_value) {

    if (current_value == 0.0f)
    {
        return RESISTANCE_MAX_VALUE;
    }

    return voltage_value / current_value;
}

uint32_t calculatePower(const uint32_t power_code, const uint32_t current_or_voltage_code) {
    uint64_t power_code_temp = static_cast<uint64_t>(power_code) * static_cast<uint64_t>(current_or_voltage_code);

    // this is left shift 13 bits to account for the 3 bit right shift in the values and the expansion from
    // 24 bit to 32 bit values
    power_code_temp = min(power_code_temp, static_cast<uint64_t>(0xffffffff) << 13);
    return static_cast<uint32_t>(power_code_temp >> 13);
}

uint32_t calculateCurrentOrVoltageFromPower(const uint32_t power_code, const uint32_t current_or_voltage_code) {
    uint64_t power_code_temp = static_cast<uint64_t>(power_code) << 13;
    return static_cast<uint32_t>(power_code_temp / current_or_voltage_code);
}

float calculatePower(const float current, const float current_or_voltage) {
    return current_or_voltage * current;
}

float bracketValue(const float value, const float maxValue) {
    return max(min(value, maxValue), 0.0f);
}

uint32_t bracketCode(const uint32_t code, const uint32_t max) {
    return min(code, max);
}

Value::Value(const ValueType type, const float value) :
_type(type),
_value(bracketValue(value, getMaxValue())),
_value_initialized(true),
_code(0), _code_initialized(false),
_format(ValueFormat::VALUE){}

Value::Value(const ValueType type, const uint32_t code) :
_type(type),
_value(0),
_value_initialized(false),
_code(bracketCode(code, getMaxCode())),
_code_initialized(true),
_format(ValueFormat::CODE) {}

float Value::getValue()
{
    if (!_value_initialized)
    {
        _value = static_cast<float>(_code) * getMinValue();
        _value_initialized = true;
    }

    return _value;
}

uint32_t Value::getCode()
{
    if (!_code_initialized)
    {
        _code = static_cast<uint32_t>(_value / getMinValue());
        _code_initialized = true;
    }

    return _code;
}

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

Power Current::operator*(Voltage& voltage)
{
    if (getFormat() == ValueFormat::VALUE && voltage.getFormat() == ValueFormat::VALUE)
    {
        return Power(calculatePower(getValue(), voltage.getValue()));
    }

    return Power(calculatePower(getCode(), voltage.getCode()));
}

Power Voltage::operator*(Current& current)
{
    if (getFormat() == ValueFormat::VALUE && current.getFormat() == ValueFormat::VALUE)
    {
        return Power(calculatePower(current.getValue(), getValue()));
    }

    return Power(calculatePower(current.getCode(), getCode()));
}

Resistance Voltage::operator/(Current& current)
{
    if (getFormat() == ValueFormat::VALUE && current.getFormat() == ValueFormat::VALUE)
    {
        return Resistance(calculateResistance(current.getValue(), getValue()));
    }

    return Resistance(calculateResistance(current.getCode(), getCode()));
}

Current Power::operator/(Voltage& voltage)
{
    return Current(calculateCurrentOrVoltageFromPower(getCode(), voltage.getCode()));
}

Voltage Power::operator/(Current& current)
{
    return Voltage(calculateCurrentOrVoltageFromPower(getCode(), current.getCode()));
}

