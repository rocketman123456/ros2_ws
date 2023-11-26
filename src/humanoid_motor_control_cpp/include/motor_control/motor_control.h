#pragma once

#include "motor_control/motor_driver.h"

namespace pi
{
    struct motor_init_info_t
    {
        can_type_t can           = CAN1;
        uint8_t    id            = 0;
        double     direction     = 1.0;
        double     offset        = 0.0;
        double     pos_scalar    = 0;
        double     vel_scalar    = 0;
        double     torque_scalar = 0;
    }

    // single motor control
    class MotorControl
    {
    public:
        MotorControl()  = default;
        ~MotorControl() = default;

        void initialize(std::shared_ptr<MotorDriver> driver, const motor_init_info_t& info);

        void setPositionDeg(double position);
        void setVelocityDeg(double velocity);
        void setPositionRad(double position);
        void setVelocityRad(double velocity);
        void setTorque(double torque);

        double getPositionRad() const;
        double getVelocityRad() const;
        double getPositionDeg() const;
        double getVelocityDeg() const;
        double getTorque() const;

    public:
        std::shared_ptr<MotorDriver> m_driver;

        can_type_t m_can           = CAN1;
        uint8_t    m_id            = 0;
        double     m_direction     = 1.0;
        double     m_offset        = 0.0;
        double     m_pos_scalar    = 0;
        double     m_vel_scalar    = 0;
        double     m_torque_scalar = 0;
    };
} // namespace pi
