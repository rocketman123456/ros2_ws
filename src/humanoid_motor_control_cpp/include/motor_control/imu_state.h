#pragma once

#include "motor_control/motor_driver.h"

#include <Eigen/Dense>
#include <math.h>
#include <vector>

namespace pi
{
    struct imu_init_info_t
    {
        double quat_scalar    = 1.0 / 32768.0;
        double angle_scalar   = 1.0 / 32768.0 * M_PI;
        double ang_vel_scalar = 1.0 / 32768.0 * 2000.0 / 180.0 * M_PI;
        double acc_scalar     = 1.0 / 32768.0 * 16.0 * 9.8;
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
