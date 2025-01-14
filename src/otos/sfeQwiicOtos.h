#pragma once

#include "math.h"
#include "sfeError.h"
#include "pico.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define PRODUCT_ID 0x5F

#define PRODUCT_IDREG 0x00
#define HARDWARE_VERSION 0x01
#define FIRMWARE VERSION 0x02

#define LINEAR_SCALAR 0x04
#define ANGULAR_SCALAR 0x05
#define IMU_CALIBRATION 0x06
#define RESET 0x07

#define SIGNAL_PROCESS_CONFIG 0x0E
#define SELF_TEST 0x0F

#define OFFSETX_L 0x10
#define OFFSETX_H 0x11
#define OFFSETY_L 0x12
#define OFFSETY_H 0x13
#define OFFSETH_L 0x14
#define OFFSETH_H 0x15

#define STATUS 0x1F

#define POSITIONX_L 0x20
#define POSITIONX_H 0x21
#define POSITIONY_L 0x22
#define POSITIONY_H 0x23
#define POSITIONH_L 0x24
#define POSITIONH_H 0x25

#define VELOCITYX_L 0x26
#define VELOCITYX_H 0x27
#define VELOCITYY_L 0x28
#define VELOCITYY_H 0x29
#define VELOCITYH_L 0x2A
#define VELOCITYH_H 0x2B

#define ACCELERATIONX_L 0x2C
#define ACCELERATIONX_H 0x2D
#define ACCELERATIONY_L 0x2E
#define ACCELERATIONY_H 0x2F
#define ACCELERATIONH_L 0x30
#define ACCELERATIONH_H 0x31

#define POSITION_STDDEVX_L 0x32
#define POSITION_STDDEVX_H 0x33
#define POSITION_STDDEVY_L 0x34
#define POSITION_STDDEVY_H 0x35
#define POSITION_STDDEVH_L 0x36
#define POSITION_STDDEVH_H 0x37

#define VELOCITY_STDDEVX_L 0x38
#define VELOCITY_STDDEVX_H 0x39
#define VELOCITY_STDDEVY_L 0x3A
#define VELOCITY_STDDEVY_H 0x3B
#define VELOCITY_STDDEVH_L 0x3C
#define VELOCITY_STDDEVH_H 0x3D

#define ACCELERATION_STDDEVX_L 0x3E
#define ACCELERATION_STDDEVX_H 0x3F
#define ACCELERATION_STDDEVY_L 0x40
#define ACCELERATION_STDDEVY_H 0x41
#define ACCELERATION_STDDEVH_L 0x42
#define ACCELERATION_STDDEVH_H 0x43

#define LSM6DSO_OUTX_L_G 0x44
#define LSM6DSO_OUTX_H_G 0x45
#define LSM6DSO_OUTY_L_G 0x46
#define LSM6DSO_OUTY_H_G 0x47
#define LSM6DSO_OUTZ_L_G 0x48
#define LSM6DSO_OUTZ_H_G 0x49

#define LSM6DSO_OUTX_L_A 0x4A
#define LSM6DSO_OUTX_H_A 0x4B
#define LSM6DSO_OUTY_L_A 0x4C
#define LSM6DSO_OUTY_H_A 0x4D
#define LSM6DSO_OUTZ_L_A 0x4E
#define LSM6DSO_OUTZ_H_A 0x4F

#define PAA5160_BURST_DATA_0 0x50
#define PAA5160_BURST_DATA_1 0x51
#define PAA5160_BURST_DATA_2 0x52
#define PAA5160_BURST_DATA_3 0x53
#define PAA5160_BURST_DATA_4 0x54
#define PAA5160_BURST_DATA_5 0x55
#define PAA5160_BURST_DATA_6 0x56
#define PAA5160_BURST_DATA_7 0x57
#define PAA5160_BURST_DATA_8 0x58
#define PAA5160_BURST_DATA_9 0x59
#define PAA5160_BURST_DATA_10 0x5A
#define PAA5160_BURST_DATA_11 0x5B

#define I2C_PORT i2c0
#define I2C_ADDR 0x69
#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1

#define MIN_SCALAR 0.872f
#define MAX_SCALAR 1.127f

// conversion factors
#define METER_TO_INCH 39.37f
#define INCH_TO_METER 0.0254f
#define RADIAN_TO_DEGREE 57.296f
#define DEGREE_TO_RADIAN 0.0174f

#define M_PI 3.14159265358979323846

// Conversion factor for the linear position registers. 16-bit signed
// registers with a max value of 10 meters (394 inches) gives a resolution
// of about 0.0003 mps (0.012 ips)
#define METER_TO_INT16 (32768.0f / 10.0f)
#define INT16_TO_METER (1.0f / METER_TO_INT16)

// Conversion factor for the linear velocity registers. 16-bit signed
// registers with a max value of 5 mps (197 ips) gives a resolution of about
// 0.00015 mps (0.006 ips)
#define MPS_TO_INT16 (32768.0f / 5.0f)
#define INT16_TO_MPS (1.0f / MPS_TO_INT16)

// Conversion factor for the linear acceleration registers. 16-bit signed
// registers with a max value of 157 mps^2 (16 g) gives a resolution of
// about 0.0048 mps^2 (0.49 mg)
#define MPSS_TO_INT16 (32768.0f / (16.0f * 9.80665f))
#define INT16_TO_MPSS (1.0f / MPSS_TO_INT16)

// Conversion factor for the angular position registers. 16-bit signed
// registers with a max value of pi radians (180 degrees) gives a resolution
// of about 0.00096 radians (0.0055 degrees)
#define RAD_TO_INT16 (32768.0f / M_PI)
#define INT16_TO_RAD (1.0f / RAD_TO_INT16)

// Conversion factor for the angular velocity registers. 16-bit signed
// registers with a max value of 34.9 rps (2000 dps) gives a resolution of
// about 0.0011 rps (0.061 degrees per second)
#define RPS_TO_INT16 (32768.0f / (2000.0f * DEGREE_TO_RADIAN))
#define INT16_TO_RPS (1.0f / RPS_TO_INT16)

// Conversion factor for the angular acceleration registers. 16-bit signed
// registers with a max value of 3141 rps^2 (180000 dps^2) gives a
// resolution of about 0.096 rps^2 (5.5 dps^2)
#define RPSS_TO_INT16 (32768.0f / (M_PI * 1000.0f))
#define INT16_TO_RPSS (1.0f / RPSS_TO_INT16)

typedef struct
{
    float x;
    float y;
    float h;
} otos_pose2d_t;

typedef enum
{
    kMeters = 0,
    kInches = 1
} linear_unit_t;

typedef enum
{
    kRadians = 0,
    kDegrees = 1
} angular_unit_t;

// version
typedef union
{
    struct
    {
        // minor version number
        uint8_t minor : 4;

        // major version number
        uint8_t major : 4;
    };

    // raw register value
    uint8_t value;
} version_t;

// signal process config
typedef union
{
    struct
    {
        // enable lookup table
        uint8_t enLut : 1;
        // enable accelerometer
        uint8_t enAcc : 1;
        // whether to rotate sensor data by heading
        uint8_t enRot : 1;
        // whether to use the variance or use 0 for all
        uint8_t enVar : 1;
        // reserved
        uint8_t reserved : 4;
    };

    // raw register value
    uint8_t value;
} signal_process_config_t;

// self test
typedef union
{
    struct
    {
        // write 1 to start self test
        uint8_t start : 1;
        // returns 1 while self test in progress
        uint8_t inProgress : 1;
        // returns 1 if self test passed
        uint8_t pass : 1;
        // returns 1 if self test failed
        uint8_t fail : 1;
        // reserved
        uint8_t reserved : 4;
    };

    // raw register value
    uint8_t value;
} self_test_config_t;

typedef union
{
    struct
    {
        // returns 1 if tilt angle threshold exceeded, while set accel data is ignored
        uint8_t warnTiltAngle : 1;
        // returns 1 if optical data is unreliable, while set only IMU data is used
        uint8_t warnOpticalTracking : 1;
        // reserved
        uint8_t reserved : 4;
        // returns 1 if the optical sensor has a fatal error
        uint8_t errorPaa : 1;
        // returns 1 if the IMU has a fatal error
        uint8_t errorLsm : 1;
    };

    // raw register value
    uint8_t value;
} otos_status_t;

class Otos
{
public:
    Otos();

    sfeError_t begin(i2c_inst_t *comm_bus = nullptr);

    sfeError_t is_connected();

    sfeError_t get_version_info(version_t &hw_version, version_t &fw_version);

    sfeError_t self_test();

    sfeError_t calibrate_imu(uint8_t num_samples = 255, bool wait_until_done = true);

    sfeError_t get_imu_calibration_progress(uint8_t &num_samples);

    linear_unit_t get_linear_unit();

    void set_linear_unit(linear_unit_t unit);

    angular_unit_t get_angular_unit();

    void set_angular_unit(angular_unit_t unit);

    sfeError_t get_linear_scalar(float &scalar);

    sfeError_t set_linear_scalar(float scalar);

    sfeError_t get_angular_scalar(float &scalar);

    sfeError_t set_angular_scalar(float scalar);

    sfeError_t reset_tracking();

    sfeError_t get_signal_process_config(signal_process_config_t &config);

    sfeError_t set_signal_process_config(signal_process_config_t &config);

    sfeError_t get_status(otos_status_t &status);

    sfeError_t get_offset(otos_pose2d_t &pose);
    sfeError_t set_offset(otos_pose2d_t &pose);

    sfeError_t get_position(otos_pose2d_t &pose);
    sfeError_t set_position(otos_pose2d_t &pose);

    sfeError_t get_velocity(otos_pose2d_t &pose);

    sfeError_t get_acceleration(otos_pose2d_t &pose);

    sfeError_t get_position_stddev(otos_pose2d_t &pose);

    sfeError_t get_velocity_stddev(otos_pose2d_t &pose);

    sfeError_t get_acceleration_stddev(otos_pose2d_t &pose);

    sfeError_t get_pos_vel_acc(otos_pose2d_t &pos, otos_pose2d_t &vel, otos_pose2d_t &acc);

    sfeError_t get_pos_vel_acc_stddev(otos_pose2d_t &pos, otos_pose2d_t &vel, otos_pose2d_t &acc);

    sfeError_t get_pos_vel_acc_and_stddev(otos_pose2d_t &pos, otos_pose2d_t &vel, otos_pose2d_t &acc, otos_pose2d_t &posstddev, otos_pose2d_t &velstddev, otos_pose2d_t &accstddev);

protected:
    sfeError_t read_pose_regs(uint8_t reg, otos_pose2d_t &pose, float raw_to_xy, float raw_to_h);

    sfeError_t write_pose_regs(uint8_t reg, otos_pose2d_t &pose, float xy_to_raw, float h_to_raw);

    void regs_to_pose(uint8_t *raw, otos_pose2d_t &pose, float raw_to_xy, float raw_to_h);

    void pose_to_regs(uint8_t *raw, otos_pose2d_t &pose, float xy_to_raw, float h_to_raw);

    i2c_inst_t *_comm_bus;

    linear_unit_t _linear_unit;
    angular_unit_t _angular_unit;

    float _meter_to_unit;
    float _rad_to_unit;
};
