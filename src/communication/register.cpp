#include "register.h"

#define MSB_32_SET(var, val) { uint8_t * d = (uint8_t *)&(val); (var) = d[3] | (d[2] << 8) | (d[1] << 16) | (d[0] << 24); }
#define MSB_24_SET(var, val) { uint8_t * d = (uint8_t *)&(val); (var) = d[2] | (d[1] << 8) | (d[0] << 16); }
#define MSB_16_SET(var, val) { (var) = (((val) & 0xFF00) >> 8) | (((val) & 0xFF) << 8); }

StatusData StatusRegister::getData(State* state) const
{
    StatusData data;
    data.voltage = *state->getVoltage();
    data.current = *state->getCurrent();
    data.temperature = state->getTemperature();
    data.load_mode = state->getMode();
    data.fault_code = state->getFaultCode();
    return data;
}

ModeData ModeRegister::getData(State* state) const
{
    ModeData data;
    data.load_mode = state->getMode();
    data.set_point = state->getSetpoint()->getCode();
    return data;
}

void ModeRegister::setData(ModeData data, State* state) const
{
    if (data.load_mode != LoadMode::OFF)
    {
        state->setSetpoint(getValue(data.load_mode, data.set_point));
    }
    state->setMode(data.load_mode);
}

DebugData DebugRegister::getData(State* state) const
{
    DebugData data;
    data.gate_control_voltage = *state->getGateControlVoltage();
    data.invalid_voltage_reading_count = state->getInvalidVoltageReadingCount();
    data.invalid_current_reading_count = state->getInvalidCurrentReadingCount();
    data.reading_count = state->getReadingCount();
    data.update_gate_voltage_count = state->getUpdateGateVoltageCount();

    return data;
}

const StatusRegister ControlLoopRegisters::STATUS;
const DebugRegister ControlLoopRegisters::DEBUG;
const ModeRegister ControlLoopRegisters::MODE;