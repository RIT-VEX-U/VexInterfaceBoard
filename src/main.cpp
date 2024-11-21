#include <Eigen/Dense>
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico.h"
#include "sr_ukf.h"
#include "spherical_conversions.h"
#include "hardware/clocks.h"

#define EIGEN_NO_DEBUG

#define N 3

int main()
{
    // the way we should go about doing this is

    // wait for an output from either imu or gps

    // if imu measurement, run predict using dt since last run
    // save timestamp

    // if gps measurement, run predict using dt since last run
    // run update using dt since last run
    // save timestamp

    // this loop continues forever

    // sr_ukf.get_xhat() returns an Eigen::Vector containing
    // x y z vx vy vz yaw pitch roll as floats

    // if possible maybe run the kalman filter on a separate thread from the glider control logic?
    // not sure whether this is possible on these microcontrollers

    // set_sys_clock_hz(300000000, true);

    Eigen::Vector<double, N> initial {0.5, 0.5, 0};
    Eigen::Vector<double, N> initial_stddevs {0.01, 0.01, 0.01};
    Eigen::Vector<double, N> process_stddevs {0.0008, 0.0008, 0.01};
    Eigen::Vector<double, ROWS> measurement_stddevs{0.002, 0.017};
    sr_ukf srukf(initial, initial_stddevs, process_stddevs, measurement_stddevs);


    stdio_init_all();
    uint64_t pretime = time_us_64();


    for (int i = 0; i < 1; i++) {
     srukf.predict(Eigen::Vector<double, 2>(0.1, 0.1), 0.1);
     srukf.update(Eigen::Vector<double, 2>(1, 1), 0.00002);
   }

   uint64_t posttime = time_us_64();

   while (true) {
    printf("%llu\n", (posttime - pretime));
   }

    // srukf.predict(Eigen::Vector<double, 6>(0.1, 0.1, 0.1, 0.1, 0.1, 0.1), 0.1);
}
