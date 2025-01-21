#pragma once

#include "sfeQwiicOtos.h"

class OtosDevice : public Otos {
    public:

    bool begin(i2c_inst_t *i2c = i2c0) {
        
        gpio_init(I2C_SDA_PIN);
        gpio_init(I2C_SCL_PIN);
        gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
        gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
        gpio_pull_up(I2C_SDA_PIN);
        gpio_pull_up(I2C_SCL_PIN);

        i2c_init(i2c, 100000);

        printf("initialized...\n");

        return Otos::begin(i2c) == kErrOkay;
    }

    private:
        i2c_inst_t i2cbus;
};