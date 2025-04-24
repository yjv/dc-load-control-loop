#ifndef PID_ALGORITHM_H
#define PID_ALGORITHM_H

#include "control_algorithm.h"
#include <PID.h>

class PidControlAlgorithm final : public ControlAlgorithm {
    public:
        PidControlAlgorithm() : PidControlAlgorithm(0, 0, 0) {}
        PidControlAlgorithm(const float kp, const float ki, const float kd)
            : gatePid(kp, ki, kd), target(Current(static_cast<uint32_t>(0)))
        {
            setTarget(target);
        }
        void addReading(Current* current, Voltage* voltage, unsigned long micros) override;
        void setTarget(Value value) override;
        GateControlVoltage* updateOutput(GateControlVoltage* output) override;
        void setCoefficients(float kp, float ki, float kd);

    private:
        arc::PID<float> gatePid;
        Value target;
};
#endif // PID_ALGORITHM_H
