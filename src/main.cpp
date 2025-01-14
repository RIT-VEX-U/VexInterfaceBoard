#include "pico/stdlib.h"
#include "pico.h"
#include <iostream>
#include <cstring>
#include "otos/OtosDevice.h"

OtosDevice otos;

int main() {
    sleep_ms(10000);

    printf("basic readings\n");

    while (otos.begin() == false) {
        printf("otos not connected, check wiring and i2c address\n");
        sleep_ms(1000);
    }

    printf("otos connected\n");
    printf("ensure otos is flat and stationary, then enter any key to calibrate imu\n");

    getchar();

    printf("calibrating imu");
    otos.calibrate_imu();
    otos.reset_tracking();

    while (true) {
        otos_pose2d_t pose;
        otos.get_position(pose);

        printf("Pose: X %f, Y %f, H %f\n");
        sleep_ms(500);
    }




    return 0;
}
