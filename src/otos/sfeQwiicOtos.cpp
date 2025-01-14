#include "sfeQwiicOtos.h"

Otos::Otos() :_comm_bus{nullptr}, _linear_unit{kMeters}, _angular_unit{kRadians}, _meter_to_unit{1}, _rad_to_unit{1} {}

sfeError_t Otos::begin(i2c_inst_t *comm_bus) {
    if (comm_bus == nullptr)
        return kErrFail;
    
    if (I2C_ADDR != I2C_ADDR)
        return kErrFail;

    _comm_bus = comm_bus;

    return is_connected();
}

sfeError_t Otos::is_connected() {
    uint8_t product_id[1];
    i2c_read_blocking(_comm_bus, PRODUCT_IDREG, product_id, 1, false);

    if (*product_id != PRODUCT_ID) {
        return kErrFail;
    }

    return kErrOkay;
}

sfeError_t Otos::get_version_info(version_t &hw_version, version_t &fw_version) {
    uint8_t raw_data[2];
    size_t read_bytes;
    
    i2c_read_blocking(_comm_bus, HARDWARE_VERSION, raw_data, 2, false);

    hw_version.value = raw_data[0];
    fw_version.value = raw_data[1];

    return kErrOkay;
}

sfeError_t Otos::self_test() {
    self_test_config_t self_test;

    self_test.start = 1;

    i2c_write_blocking(_comm_bus, SELF_TEST, &self_test.value, 1, true);

    for (int i = 0; i < 10; i++) {
        sleep_ms(5);

        i2c_read_blocking(_comm_bus, SELF_TEST, &self_test.value, 1, false);

        if (self_test.inProgress == 0) {
            break;
        }
    }

    return (self_test.pass == 1) ? kErrOkay : kErrFail;

}

sfeError_t Otos::calibrate_imu(uint8_t num_samples, bool wait_until_done) {
    i2c_write_blocking(_comm_bus, IMU_CALIBRATION, &num_samples, 1, true);

    sleep_ms(3);

    if (!wait_until_done) {
        return kErrOkay;
    }

    for (uint8_t num_attempts = num_samples; num_attempts > 0; num_attempts--) {
        uint8_t calibration_value;
        i2c_read_blocking(_comm_bus, IMU_CALIBRATION, &calibration_value, 1, false);

        if (calibration_value == 0) {
            return kErrOkay;
        }

        sleep_ms(3);
    }

    // only happens if the max attempts is reached
    return kErrFail;
    
}

sfeError_t Otos::get_imu_calibration_progress(uint8_t &num_samples) {
    int err = i2c_read_blocking(_comm_bus, IMU_CALIBRATION, &num_samples, 1, false);
    return err == -1 ? kErrFail : kErrOkay;
}

linear_unit_t Otos::get_linear_unit() {
    return _linear_unit;
}

void Otos::set_linear_unit(linear_unit_t unit) {
    if (unit == _linear_unit) {
        return;
    }

    _linear_unit = unit;

    _meter_to_unit = (unit == kMeters) ? 1.0f : METER_TO_INCH;
}

angular_unit_t Otos::get_angular_unit() {
    return _angular_unit;
}

void Otos::set_angular_unit(angular_unit_t unit) {
    if (unit == _angular_unit) {
        return;
    }

    _angular_unit = unit;

    _rad_to_unit = (unit == kRadians) ? 1.0f : RADIAN_TO_DEGREE;
}

sfeError_t Otos::get_linear_scalar(float &scalar) {
    uint8_t raw_scalar;
    int err = i2c_read_blocking(_comm_bus, LINEAR_SCALAR, &raw_scalar, 1, false);
    if (err == -1)
        return kErrFail;

    scalar = (((int8_t)raw_scalar) * 0.001f) + 1.0f;

    return kErrOkay;
}

sfeError_t Otos::set_linear_scalar(float scalar) {
    if (scalar < MIN_SCALAR || scalar > MAX_SCALAR)
        return kErrFail;

    uint8_t raw_scalar = (int8_t)((scalar - 1.0f) * 1000 + 0.5f);

    int err = i2c_write_blocking(_comm_bus, LINEAR_SCALAR, &raw_scalar, 1, true);

    return err == -1 ? kErrFail : kErrOkay;
}

sfeError_t Otos::get_angular_scalar(float &scalar) {
    uint8_t raw_scalar;
    int err = i2c_read_blocking(_comm_bus, ANGULAR_SCALAR, &raw_scalar, 1, false);
    if (err == -1)
        return kErrFail;

    scalar = (((int8_t)raw_scalar) * 0.001f) + 1.0f;

    return kErrOkay;
}

sfeError_t Otos::set_angular_scalar(float scalar) {
    if (scalar < MIN_SCALAR || scalar > MAX_SCALAR)
        return kErrFail;

    uint8_t raw_scalar = (int8_t)((scalar - 1.0f) * 1000 + 0.5f);

    int err = i2c_write_blocking(_comm_bus, ANGULAR_SCALAR, &raw_scalar, 1, true);

    return err == -1 ? kErrFail : kErrOkay;
}

sfeError_t Otos::reset_tracking() {
    uint8_t reset_flag = 0x01;
    int err = i2c_write_blocking(_comm_bus, RESET, &reset_flag, 1, true);
    
    return err == -1 ? kErrFail : kErrOkay;
}

sfeError_t Otos::get_signal_process_config(signal_process_config_t &config) {
    int err = i2c_read_blocking(_comm_bus, SIGNAL_PROCESS_CONFIG, &config.value, 1, false);
    return err == -1 ? kErrFail : kErrOkay;
}

sfeError_t Otos::set_signal_process_config(signal_process_config_t &config) {
    int err = i2c_write_blocking(_comm_bus, SIGNAL_PROCESS_CONFIG, &config.value, 1, true);
    return err == -1 ? kErrFail : kErrOkay;
}

sfeError_t Otos::get_status(otos_status_t &status) {
    int err = i2c_read_blocking(_comm_bus, STATUS, &status.value, 1, false);
    return err == -1 ? kErrFail : kErrOkay;
}

sfeError_t Otos::get_offset(otos_pose2d_t &pose) {
    return read_pose_regs(OFFSETX_L, pose, INT16_TO_METER, INT16_TO_RAD);
}

sfeError_t Otos::set_offset(otos_pose2d_t &pose) {
    return write_pose_regs(OFFSETX_L, pose, METER_TO_INT16, RAD_TO_INT16);
}

sfeError_t Otos::get_position(otos_pose2d_t &pose) {
    return read_pose_regs(POSITIONX_L, pose, INT16_TO_METER, INT16_TO_RAD);
}

sfeError_t Otos::set_position(otos_pose2d_t &pose) {
    return write_pose_regs(POSITIONX_L, pose, METER_TO_INT16, RAD_TO_INT16);
}

sfeError_t Otos::get_velocity(otos_pose2d_t &pose) {
    return read_pose_regs(VELOCITYX_L, pose, INT16_TO_MPS, INT16_TO_RPS);
}

sfeError_t Otos::get_acceleration(otos_pose2d_t &pose) {
    return read_pose_regs(ACCELERATIONX_L, pose, INT16_TO_MPSS, INT16_TO_RPSS);
}

sfeError_t Otos::get_position_stddev(otos_pose2d_t &pose) {
    return read_pose_regs(POSITION_STDDEVX_L, pose, INT16_TO_METER, INT16_TO_RAD);
}

sfeError_t Otos::get_velocity_stddev(otos_pose2d_t &pose) {
    return read_pose_regs(VELOCITY_STDDEVX_L, pose, INT16_TO_MPS, INT16_TO_RPS);
}

sfeError_t Otos::get_acceleration_stddev(otos_pose2d_t &pose) {
    return read_pose_regs(ACCELERATION_STDDEVX_L, pose, INT16_TO_MPSS, INT16_TO_RPSS);
}

sfeError_t Otos::get_pos_vel_acc(otos_pose2d_t &pos, otos_pose2d_t &vel, otos_pose2d_t &acc) {
    uint8_t raw[18];
    size_t bytes_read;
    int err = i2c_read_blocking(_comm_bus, POSITIONX_L, raw, 18, false);

    if (err == -1 || err != 18) {
        return kErrFail;
    }

    regs_to_pose(raw, pos, INT16_TO_METER, INT16_TO_RAD);
    regs_to_pose(raw + 6, vel, INT16_TO_MPS, INT16_TO_RPS);
    regs_to_pose(raw + 12, acc, INT16_TO_MPSS, INT16_TO_RPSS);

    return kErrOkay;
}

sfeError_t Otos::get_pos_vel_acc_stddev(otos_pose2d_t &pos, otos_pose2d_t &vel, otos_pose2d_t &acc) {
    uint8_t raw[18];
    size_t bytes_read;
    int err = i2c_read_blocking(_comm_bus, POSITION_STDDEVX_L, raw, 18, false);

    if (err == -1 || err != 18) {
        return kErrFail;
    }

    regs_to_pose(raw, pos, INT16_TO_METER, INT16_TO_RAD);
    regs_to_pose(raw + 6, vel, INT16_TO_MPS, INT16_TO_RPS);
    regs_to_pose(raw + 12, acc, INT16_TO_MPSS, INT16_TO_RPSS);

    return kErrOkay;
}

sfeError_t Otos::get_pos_vel_acc_and_stddev(otos_pose2d_t &pos, otos_pose2d_t &vel, otos_pose2d_t &acc, otos_pose2d_t &posstddev, otos_pose2d_t &velstddev, otos_pose2d_t &accstddev) {
    uint8_t raw[36];
    size_t bytes_read;
    int err = i2c_read_blocking(_comm_bus, POSITIONX_L, raw, 36, false);

    if (err == -1 || err != 36) {
        return kErrFail;
    }

    regs_to_pose(raw, pos, INT16_TO_METER, INT16_TO_RAD);
    regs_to_pose(raw + 6, vel, INT16_TO_MPS, INT16_TO_RPS);
    regs_to_pose(raw + 12, acc, INT16_TO_MPSS, INT16_TO_RPSS);
    regs_to_pose(raw + 18, posstddev, INT16_TO_METER, INT16_TO_RAD);
    regs_to_pose(raw + 24, velstddev, INT16_TO_MPS, INT16_TO_RPS);
    regs_to_pose(raw + 30, accstddev, INT16_TO_MPSS, INT16_TO_RPSS);

    return kErrOkay;
}

sfeError_t Otos::read_pose_regs(uint8_t reg, otos_pose2d_t &pose, float raw_to_xy, float raw_to_h) {
    size_t bytes_read;
    uint8_t raw_data[6];

    int err = i2c_read_blocking(_comm_bus, reg, raw_data, 6, false);
    if (err == -1 || err != 6)
        return kErrFail;

    regs_to_pose(raw_data, pose, raw_to_xy, raw_to_h);

    return kErrOkay;
        
}

sfeError_t Otos::write_pose_regs(uint8_t reg, otos_pose2d_t &pose, float xy_to_raw, float h_to_raw) {
    uint8_t raw_data[6];
    pose_to_regs(raw_data, pose, xy_to_raw, h_to_raw);

    int err = i2c_write_blocking(_comm_bus, reg, raw_data, 6, true);
    return err == -1 ? kErrFail : kErrOkay;
}

void Otos::regs_to_pose(uint8_t *raw, otos_pose2d_t &pose, float raw_to_xy, float raw_to_h) {
    int16_t raw_x = (raw[1] << 8) | raw[0];
    int16_t raw_y = (raw[3] << 8) | raw[2];
    int16_t raw_h = (raw[5] << 8) | raw[4];

    pose.x = raw_x * raw_to_xy * _meter_to_unit;
    pose.y = raw_y * raw_to_xy * _meter_to_unit;
    pose.h = raw_h * raw_to_h * _rad_to_unit;
}

void Otos::pose_to_regs(uint8_t *raw, otos_pose2d_t &pose, float xy_to_raw, float h_to_raw) {
    int16_t rawx = pose.x * xy_to_raw / _meter_to_unit;
    int16_t rawy = pose.y * xy_to_raw / _meter_to_unit;
    int16_t rawh = pose.h * h_to_raw / _rad_to_unit;

    raw[0] = rawx & 0xFF;
    raw[1] = (rawx >> 8) & 0xFF;
    raw[2] = rawy & 0xFF;
    raw[3] = (rawy >> 8) & 0xFF;
    raw[4] = rawh & 0xFF;
    raw[5] = (rawh >> 8) & 0xFF;
}
