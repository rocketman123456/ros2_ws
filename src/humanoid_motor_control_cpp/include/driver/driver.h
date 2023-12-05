#pragma once

namespace pi
{
    class IDriver
    {
    public:
        virtual ~IDriver() = default;

        virtual void initialize() = 0;
        virtual void finalize()   = 0;

        virtual void set_motor_position(int8_t motor_id, int32_t position)                                                       = 0;
        virtual void set_motor_position(int8_t motor_id, int32_t position, int32_t velocity, int32_t torque, float kp, float kd) = 0;
        virtual void set_motor_velocity(int8_t motor_id, int32_t velocity)                                                       = 0;
        virtual void set_motor_torque(int8_t motor_id, int32_t torque)                                                           = 0;
    };
} // namespace pi
