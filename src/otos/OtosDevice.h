#pragma once

#include "sfeQwiicOtos.h"

class OtosDevice : public Otos {
    public:

    bool begin(i2c_inst_t *i2c = i2c0) {
        i2c_init(i2c, 100000);

        return Otos::begin(i2c) == kErrOkay;
    }

    private:
        i2c_inst_t i2cbus;
};