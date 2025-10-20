/**
 * @file register.cpp
 * @brief Register implementation for I2C communication
 *
 * This file implements the register classes for I2C communication with the
 * electronic load. It provides methods to read and write different types of
 * data (status, mode, debug) from the load system.
 */

#include "register.h"

// Byte order conversion macros for network communication
#define MSB_32_SET(var, val) { uint8_t * d = (uint8_t *)&(val); (var) = d[3] | (d[2] << 8) | (d[1] << 16) | (d[0] << 24); }
#define MSB_24_SET(var, val) { uint8_t * d = (uint8_t *)&(val); (var) = d[2] | (d[1] << 8) | (d[0] << 16); }
#define MSB_16_SET(var, val) { (var) = (((val) & 0xFF00) >> 8) | (((val) & 0xFF) << 8); }

namespace communication
{

    /**
     * @brief Get status data from the load
     * @param load Pointer to the load system
     * @return StatusData containing current load status
     *
     * Retrieves the current status information from the load system,
     * including voltage, current, temperature, load mode, and fault code.
     */
    StatusData StatusRegister::getData(load::Load* load) const
    {
        StatusData data;
        data.voltage = *load->getState()->getVoltage();
        data.current = *load->getState()->getCurrent();
        data.temperature = load->getState()->getTemperature();
        data.load_mode = load->getState()->getMode();
        data.fault_code = load->getState()->getFaultCode();
        return data;
    }

    /**
     * @brief Get mode data from the load
     * @param load Pointer to the load system
     * @return ModeData containing current mode and setpoint
     *
     * Retrieves the current operating mode and setpoint from the load system.
     * The setpoint is returned as a raw code value for network transmission.
     */
    ModeData ModeRegister::getData(load::Load* load) const
    {
        ModeData data;
        data.load_mode = load->getState()->getMode();
        data.set_point = load->getState()->getSetpoint()->getCode();
        return data;
    }

    /**
     * @brief Set mode data in the load
     * @param data ModeData containing new mode and setpoint
     * @param load Pointer to the load system
     *
     * Updates the load's operating mode and setpoint based on the provided data.
     * If the mode is OFF, the setpoint is automatically set to 0A current.
     * The setpoint is created from the raw code value received over the network.
     */
    void ModeRegister::setData(ModeData data, load::Load* load) const
    {
        if (data.load_mode != LoadMode::OFF)
        {
            // Create appropriate Value object from mode and setpoint code
            load->setSetpoint(load::getValue(data.load_mode, data.set_point));
        }
        // Set the load mode (this will set setpoint to 0A if mode is OFF)
        load->setMode(data.load_mode);
    }

    /**
     * @brief Get debug data from the load
     * @param load Pointer to the load system
     * @return DebugData containing debug information
     *
     * Retrieves debug information from the load system, including
     * gate control voltage, error counts, and operational statistics.
     */
    DebugData DebugRegister::getData(load::Load* load) const
    {
        DebugData data;
        data.gate_control_voltage = *load->getState()->getGateControlVoltage();
        data.invalid_voltage_reading_count = load->getState()->getInvalidVoltageReadingCount();
        data.invalid_current_reading_count = load->getState()->getInvalidCurrentReadingCount();
        data.reading_count = load->getState()->getReadingCount();
        data.update_gate_voltage_count = load->getState()->getUpdateGateVoltageCount();

        return data;
    }

    // Static register instances
    const StatusRegister ControlLoopRegisters::STATUS;
    const DebugRegister ControlLoopRegisters::DEBUG;
    const ModeRegister ControlLoopRegisters::MODE;
}