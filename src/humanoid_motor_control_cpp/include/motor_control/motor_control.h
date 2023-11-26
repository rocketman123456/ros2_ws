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

        void initialize(const motor_init_info_t& info);

        void setPositionDeg(std::shared_ptr<MotorDriver> driver, double position);
        void setVelocityDeg(std::shared_ptr<MotorDriver> driver, double velocity);
        void setPositionRad(std::shared_ptr<MotorDriver> driver, double position);
        void setVelocityRad(std::shared_ptr<MotorDriver> driver, double velocity);
        void setTorque(std::shared_ptr<MotorDriver> driver, double torque);

        // get motor data from driver
        void update(std::shared_ptr<MotorDriver> driver);

        double getPositionRad() const;
        double getVelocityRad() const;
        double getPositionDeg() const;
        double getVelocityDeg() const;
        double getTorque() const;

    private:
        can_type_t m_can           = CAN1;
        uint8_t    m_id            = 0;
        double     m_direction     = 1.0;
        double     m_offset        = 0.0;
        double     m_pos_scalar    = 0;
        double     m_vel_scalar    = 0;
        double     m_torque_scalar = 0;

        double m_position = 0.0;
        double m_velocity = 0.0;
        double m_torque   = 0.0;
    };
} // namespace pi
