//
// Created by Joseph Deray on 4/10/25.
//

#include "control_loop.h"

#include <shared/communication/data.h>
#include "register.h"

void ControlLoop::onRequest()
{
    size_t size;

    switch (_current_register)
    {
    case Registers::STATUS.ADDRESS:
        {
            StatusData status_data = ControlLoopRegisters::STATUS.getData(&_state);
            size = StatusData::SIZE;
            status_data.toNetwork(Buffer(_buffer, size));
        }
        break;
    case Registers::MODE.ADDRESS:
        {
            ModeData mode_data = ControlLoopRegisters::MODE.getData(&_state);
            size = ModeData::SIZE;
            mode_data.toNetwork(Buffer(_buffer, size));
        }
        break;
    case Registers::DEBUG.ADDRESS:
        {
            DebugData debug_data = ControlLoopRegisters::DEBUG.getData(&_state);
            size = DebugData::SIZE;
            debug_data.toNetwork(Buffer(_buffer, size));
        }
        break;
    default:
        ESP_LOGE("ControlLoop", "Unknown register: %d", _current_register);
        abort();
    }

    _protocol->write(_buffer, size);
}

void ControlLoop::onReceive(const int num_bytes)
{
    _protocol->readBytes(_buffer, num_bytes);
    _current_register = _buffer[0];

    if (num_bytes == 1)
    {
        return;
    }

    const size_t size = num_bytes - 1;

    switch (_current_register)
    {
    case Registers::MODE.ADDRESS:
        {
            ModeData mode_data;
            mode_data.fromNetwork(Buffer(_buffer + 1, size));
            ControlLoopRegisters::MODE.setData(mode_data, &_state);
        }
        break;
    default:
        ESP_LOGE("ControlLoop", "Unknown register: %d", _current_register);
        abort();
    }
}
