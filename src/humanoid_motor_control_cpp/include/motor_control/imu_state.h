#pragma once

#include "motor_control/motor_driver.h"

namespace pi
{
    struct imu_init_info_t
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
    class ImuState
    {
    public:
        ImuState(const imu_init_info_t& info);
        ~ImuState() = default;

        // get motor data from driver
        void update(std::shared_ptr<MotorDriver> driver);

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