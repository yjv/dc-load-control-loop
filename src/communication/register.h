//
// Created by Joseph Deray on 4/8/25.
//

#ifndef REGISTER_H
#define REGISTER_H

#include <shared/communication/data.h>
#include <load.h>

template<typename D>
class ReadRegister
{
public:
    virtual ~ReadRegister() = default;
    virtual D getData(State* state) const = 0;
};

template<typename D>
class ReadWriteRegister: public ReadRegister<D>
{
public:
    ~ReadWriteRegister() override = default;
    virtual void setData(D data, State* state) const = 0;
};

class StatusRegister final : public ReadRegister<StatusData> {
public:
    StatusData getData(State* state) const override;
};

class DebugRegister final : public ReadRegister<DebugData> {
public:
    DebugData getData(State* state) const override;
};

class ModeRegister final : public ReadWriteRegister<ModeData> {
public:
    ModeData getData(State* state) const override;
    void setData(ModeData data, State* state) const override;
};

class ControlLoopRegisters {
public:
    static const StatusRegister STATUS;
    static const DebugRegister DEBUG;
    static const ModeRegister MODE;
};

#endif //REGISTER_H
