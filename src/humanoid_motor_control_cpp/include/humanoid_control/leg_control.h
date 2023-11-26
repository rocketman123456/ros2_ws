#pragma once
#include "motor_control/motor_control.h"
#include "motor_control/motor_driver.h"

#include "modern_robotics/modern_robotics.h"

#include <vector>

#include <Eigen/Eigen>

namespace pi
{
    class LegControl
    {
    public:
        LegControl()  = default;
        ~LegControl() = default;

        void initialize(std::shared_ptr<MotorDriver> driver, const std::vector<motor_init_info_t>& infos);
        void reset();

        void positionControlDeg(const std::vector<double>& angles);
        void positionControlRad(const std::vector<double>& angles);
        void torqueControl(const std::vector<double>& torques);

        void poistionControl(const Eigen::Matrix4d& target);
        void forceControl(const Eigen::VectorXd& force);

    public:
        std::shared_ptr<MotorDriver> m_driver;
        std::vector<MotorControl>    m_motors;
    };
} // namespace pi
