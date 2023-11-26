#pragma once

#include <cstddef>
#include <cstdint>


namespace pi
{
#pragma pack(1)
    struct imu_t
    {
        int16_t acc_x {0};
        int16_t acc_y {0};
        int16_t acc_z {0};
        int16_t ang_vel_x {0};
        int16_t ang_vel_y {0};
        int16_t ang_vel_z {0};
        int16_t angle_roll {0};
        int16_t angle_pitch {0};
        int16_t angle_yaw {0};
        int16_t mag_x {0};
        int16_t mag_y {0};
        int16_t mag_z {0};
    };

    struct motor_t
    {
        uint8_t motor_id {0};
        int32_t motor_cmd {0};
        int32_t position {0};
        int32_t velocity {0};
        int32_t torque {0};
    };

    struct motor_set_t
    {
        uint8_t motor_id {0};
        uint8_t motor_cmd {0};
        int32_t position {0};
        int32_t velocity {0};
        int32_t torque {0};
        float   kp {0};
        float   kd {0};
    };
#pragma pack()

    enum can_type_t
    {
        CAN1 = 0x10,
        CAN2 = 0x20,
        CAN3 = 0x30,
        CAN4 = 0x40,
    };

    struct motor_status_t
    {
        motor_t motor_fb1[14] {};
        motor_t motor_fb2[14] {};

        imu_t imu_data {};

        uint8_t foot_sensor1[3] {0};
        uint8_t foot_sensor2[3] {0};
    };

    constexpr uint32_t k_motor_type = 0;

    constexpr uint32_t k_can1_num           = 9;
    constexpr uint32_t k_can2_num           = 9;
    constexpr uint32_t k_enable_imu         = 1;
    constexpr uint32_t k_enable_foot_sensor = 1;
    constexpr uint32_t k_enable_stop        = 0;

    constexpr uint32_t k_motor_set_len    = sizeof(motor_set_t);
    constexpr uint32_t k_motor_status_len = sizeof(motor_t);
    constexpr uint32_t k_yj901s_data_len  = sizeof(imu_t);
    constexpr uint32_t k_data_pkg_size    = 3 + (k_can1_num + k_can2_num) * k_motor_set_len + k_enable_imu * k_yj901s_data_len + k_enable_foot_sensor * 6;

    constexpr uint32_t k_foot_sensor_id_1 = 0x01;
    constexpr uint32_t k_foot_sensor_id_2 = 0x02;

    constexpr uint32_t k_spi_speed = 6000000;

    constexpr int32_t k_error_transfer_data = 0xffffaaaa;
} // namespace pi