#include "humanoid_control/leg_control.h"

#include <cassert>

namespace pi
{
    void LegControl::initialize(const std::vector<motor_init_info_t>& infos)
    {
        m_motors.resize(infos.size());
        for (size_t i = 0; i < infos.size(); ++i)
        {
            m_motors[i].initialize(infos[i]);
        }
    }

    void LegControl::positionControlDeg(std::shared_ptr<SpiDriver> driver, const std::vector<double>& angles)
    {
        assert(angles.size() == m_motors.size());
        for (size_t i = 0; i < m_motors.size(); ++i)
        {
            m_motors[i].setPositionDeg(driver, angles[i]);
        }
    }

    void LegControl::positionControlRad(std::shared_ptr<SpiDriver> driver, const std::vector<double>& angles)
    {
        assert(angles.size() == m_motors.size());
        for (size_t i = 0; i < m_motors.size(); ++i)
        {
            m_motors[i].setPositionRad(driver, angles[i]);
        }
    }

    void LegControl::positionControlWithAnkleIKDeg(std::shared_ptr<SpiDriver> driver, const std::vector<double>& angles)
    {
        // TODO :
        assert(angles.size() == m_motors.size());
        for (size_t i = 0; i < m_motors.size(); ++i)
        {
            m_motors[i].setPositionDeg(driver, angles[i]);
        }
    }

    void LegControl::positionControlWithAnkleIKRad(std::shared_ptr<SpiDriver> driver, const std::vector<double>& angles)
    {
        // TODO :
        assert(angles.size() == m_motors.size());
        for (size_t i = 0; i < m_motors.size(); ++i)
        {
            m_motors[i].setPositionRad(driver, angles[i]);
        }
    }

    void LegControl::torqueControl(std::shared_ptr<SpiDriver> driver, const std::vector<double>& torques)
    {
        assert(torques.size() == m_motors.size());
        for (size_t i = 0; i < m_motors.size(); ++i)
        {
            m_motors[i].setTorque(driver, torques[i]);
        }
    }

    void LegControl::poistionControl(std::shared_ptr<SpiDriver> driver, const Eigen::Matrix4d& target)
    {
        // TODO :
        // assert(angles.size() == m_motors.size());
        // for (size_t i = 0; i < m_motors.size(); ++i)
        // {
        //     m_motors[i].setPositionDeg(driver, angles[i]);
        // }
    }

    void LegControl::forceControl(std::shared_ptr<SpiDriver> driver, const Eigen::VectorXd& force)
    {
        // TODO :
        // assert(angles.size() == m_motors.size());
        // for (size_t i = 0; i < m_motors.size(); ++i)
        // {
        //     m_motors[i].setPositionDeg(driver, angles[i]);
        // }
    }

    MotorControl* LegControl::getMotor(int32_t index)
    {
        assert(index >= 0 && index < m_motors.size());
        return &m_motors[index];
    }
} // namespace pi
