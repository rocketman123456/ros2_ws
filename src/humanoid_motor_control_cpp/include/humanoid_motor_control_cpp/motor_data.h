#pragma once
#include "humanoid_motor_control_cpp/config.h"

#include <cstdint>

namespace pi
{
    struct alignas(1) motor_t
    {
        uint8_t motor_id {0};
        int32_t motor_cmd {0};
        int32_t position {0};
        int32_t velocity {0};
        int32_t torque {0};
    };

    struct alignas(1) motor_data_t
    {
        union
        {
            motor_t motor;
            uint8_t data[MOTOR_STATUS_LENGTH];
        };
    };

    struct alignas(1) imu_t
    {
        int16_t accX {0};
        int16_t accY {0};
        int16_t accZ {0};
        int16_t angVelX {0};
        int16_t angVelY {0};
        int16_t angVelZ {0};
        int16_t angle_roll {0};
        int16_t angle_pitch {0};
        int16_t angle_yaw {0};
        int16_t magX {0};
        int16_t magY {0};
        int16_t magZ {0};
    };

    struct alignas(1) imu_data_t
    {
        union
        {
            imu_t   imu_data;
            uint8_t data[YJ901S_DATA_SIZE];
        };
    };

    struct alignas(1) motor_set_t
    {
        uint8_t motor_id;
        uint8_t motor_cmd;
        int32_t position;
        int32_t velocity;
        int32_t torque;
        float   kp;
        float   kd;
    };

    struct alignas(1) motor_set_data_t
    {
        union
        {
            motor_set_t motor;
            uint8_t     data[MOTOR_SET_LENGTH];
        };
    };

    enum tranfer_send_type_e
    {
        ANG2POS = 0x20,
        RAD2POS,
        TOR2TOR,
    };

    enum tranfer_rec_type_e
    {
        POS2ANG = 0x30,
        POS2RAD,
    };

    struct motor_status_t
    {
        motor_data_t motor_fb1[14] = {};
        motor_data_t motor_fb2[14] = {};

        imu_data_t imu_data = {};

        uint8_t foot_sensor1[3] = {0};
        uint8_t foot_sensor2[3] = {0};
    };
} // namespace pi