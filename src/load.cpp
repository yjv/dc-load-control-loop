#include "load.h"

LoadMode getLoadMode(ValueType type)
{
    switch (type) {
    case ValueType::CURRENT:
        return LoadMode::CONSTANT_CURRENT;
    case ValueType::VOLTAGE:
        return LoadMode::CONSTANT_VOLTAGE;
    case ValueType::POWER:
        return LoadMode::CONSTANT_POWER;
    case ValueType::RESISTANCE:
        return LoadMode::CONSTANT_RESISTANCE;
    default:
        // Log the unexpected type and abort
            ESP_LOGE("ValueType", "Unexpected ValueType: %d", static_cast<int>(type));
        abort();
    }
}

ValueType getValueType(LoadMode mode)
{
    switch (mode)
    {
    case LoadMode::CONSTANT_CURRENT:
        return ValueType::CURRENT;
    case LoadMode::CONSTANT_VOLTAGE:
        return ValueType::VOLTAGE;
    case LoadMode::CONSTANT_POWER:
        return ValueType::POWER;
    case LoadMode::CONSTANT_RESISTANCE:
        return ValueType::RESISTANCE;
    default:
        // Log the unexpected mode and abort
            ESP_LOGE("LoadMode", "Unexpected LoadMode: %d", static_cast<int>(mode));
        abort();
    }
}

Value* getValue(LoadMode mode, uint32_t value)
{
    switch (mode)
    {
    case LoadMode::CONSTANT_CURRENT:
        return new Current(value);
    case LoadMode::CONSTANT_VOLTAGE:
        return new Voltage(value);
    case LoadMode::CONSTANT_POWER:
        return new Power(value);
    case LoadMode::CONSTANT_RESISTANCE:
        return new Resistance(value);
    default:
        // Log the unexpected mode and abort
        ESP_LOGE("LoadMode", "Unexpected LoadMode: %d", static_cast<int>(mode));
        abort();
    }
}
