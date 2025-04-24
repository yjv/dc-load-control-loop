#include "pid_algorithm.h"

void PidControlAlgorithm::addReading(Current* current, Voltage* voltage, const unsigned long micros)
{
    float input = 0;
    float target = this->target.getValue();

    switch (this->target.getType())
    {
    case ValueType::CURRENT:
        input = current->getValue();
        break;
    case ValueType::VOLTAGE:
        input = voltage->getValue();
        break;
    case ValueType::POWER:
        target = this->target.getValue() / std::max(voltage->getValue(), voltage->getMinValue());
        input = current->getValue();
        break;
    case ValueType::RESISTANCE:
        target = voltage->getValue() / std::max(this->target.getValue(), this->target.getMinValue());
        input = current->getValue();
        break;
    default:
        ESP_LOGE("PidControlAlgorithm", "Unhandled ValueType in PidControlAlgorithm::addReading");
        std::terminate();
    }

    gatePid.setTarget(target);
    gatePid.setInput(input, micros);
}

void PidControlAlgorithm::setTarget(Value target)
{
    if (target.getType() != this->target.getType())
    {
        gatePid = arc::PID<float>(gatePid.getKp(), gatePid.getKi(), gatePid.getKd());
    }

    this->target = target;
    gatePid.setTarget(target.getValue());
}

GateControlVoltage* PidControlAlgorithm::updateOutput(GateControlVoltage* currentOutput)
{
    const float delta = gatePid.getOutput();
    float control_voltage_value = currentOutput->getValue();

    if (this->target.getType() != ValueType::VOLTAGE)
    {
      control_voltage_value += delta;
    }
    else
    {
      control_voltage_value -= delta;
    }

    return new GateControlVoltage(control_voltage_value);
}

void PidControlAlgorithm::setCoefficients(const float kp, const float ki, const float kd)
{
    gatePid.setKp(kp);
    gatePid.setKi(ki / 1000.0f);
    gatePid.setKd(kd * 1000.0f);
}
