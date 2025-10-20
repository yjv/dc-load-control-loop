#include "load.h"

/**
 * @file load.cpp
 * @brief Implementation of electronic load control system
 * 
 * This file contains the implementation of utility functions for converting
 * between ValueType and LoadMode, and creating Value objects from LoadMode.
 */

namespace load
{
    /**
     * @brief Convert ValueType to LoadMode
     * @param type ValueType to convert
     * @return Corresponding LoadMode
     * 
     * Converts a ValueType to its corresponding LoadMode.
     * Aborts if an unexpected ValueType is encountered.
     */
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

    /**
     * @brief Convert LoadMode to ValueType
     * @param mode LoadMode to convert
     * @return Corresponding ValueType
     * 
     * Converts a LoadMode to its corresponding ValueType.
     * Aborts if an unexpected LoadMode is encountered.
     */
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

    /**
     * @brief Create Value object from LoadMode and value
     * @param mode LoadMode to create value for
     * @param value Raw value code
     * @return Pointer to new Value object
     * 
     * Creates a new Value object of the appropriate type based on the LoadMode.
     * The value parameter is the raw code value for the specific value type.
     * Aborts if an unexpected LoadMode is encountered.
     */
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
}
