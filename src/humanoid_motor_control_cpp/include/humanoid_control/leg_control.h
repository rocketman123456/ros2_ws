#pragma once
#include "driver/spi_driver.h"
#include "low_level_control/motor_control.h"
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

        void initialize(const std::vector<motor_init_info_t>& infos);

        void positionControlDeg(std::shared_ptr<SpiDriver> driver, const std::vector<double>& angles);
        void positionControlRad(std::shared_ptr<SpiDriver> driver, const std::vector<double>& angles);
        void positionControlWithAnkleIKDeg(std::shared_ptr<SpiDriver> driver, const std::vector<double>& angles);
        void positionControlWithAnkleIKRad(std::shared_ptr<SpiDriver> driver, const std::vector<double>& angles);
        void torqueControl(std::shared_ptr<SpiDriver> driver, const std::vector<double>& torques);

        void poistionControl(std::shared_ptr<SpiDriver> driver, const Eigen::Matrix4d& target);
        void forceControl(std::shared_ptr<SpiDriver> driver, const Eigen::VectorXd& force);

        MotorControl* getMotor(int32_t index);

    public:
        std::vector<MotorControl>  m_motors;

        // special parameters for current robot control
        double d;
        double L1;
        double h1;
        double h2;
    };
} // namespace pi
