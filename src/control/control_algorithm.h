#ifndef CONTROL_ALGORITHM_H
#define CONTROL_ALGORITHM_H

#include <cstdint>
#include <shared/value.h>

class ControlAlgorithm {
    public:
        virtual ~ControlAlgorithm() = default; // Virtual destructor is important
        virtual void addReading(Current* current, Voltage* voltage, unsigned long micros) = 0;
        virtual void setTarget(Value value) = 0;
        virtual GateControlVoltage* updateOutput(GateControlVoltage* output) = 0;
};

#endif // CONTROL_ALGORITHM_H