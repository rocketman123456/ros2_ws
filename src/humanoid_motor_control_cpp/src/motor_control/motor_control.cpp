#include "motor_control/motor_control.h"

#define M_PI 3.14159265358979323846 /* pi */

namespace pi
{
    void MotorControl::initialize(std::shared_ptr<MotorDriver> driver, const motor_init_info_t& info)
    {
        m_driver        = driver;
        m_can           = info.can;
        m_id            = info.id;
        m_direction     = info.direction;
        m_offset        = info.offset;
        m_pos_scalar    = info.pos_scalar;
        m_vel_scalar    = info.vel_scalar;
        m_torque_scalar = info.torque_scalar;
    }

    void MotorControl::setPositionDeg(double position)
    {
        double  result  = position * m_direction * m_pos_scalar * M_PI / 180.0;
        uint8_t temp_id = m_can | (motor_id + 1);
        m_driver->set_motor_position(temp_id, result);
    }

    void MotorControl::setVelocityDeg(double velocity)
    {
        double  result  = velocity * m_direction * m_vel_scalar * M_PI / 180.0;
        uint8_t temp_id = m_can | (motor_id + 1);
        m_driver->set_motor_velocity(temp_id, result);
    }

    void MotorControl::setPositionRad(double position)
    {
        double  result  = position * m_direction * m_pos_scalar;
        uint8_t temp_id = m_can | (motor_id + 1);
        m_driver->set_motor_position(temp_id, result);
    }

    void MotorControl::setVelocityRad(double velocity)
    {
        double  result  = torque * m_direction * m_vel_scalar;
        uint8_t temp_id = m_can | (motor_id + 1);
        m_driver->set_motor_velocity(temp_id, result);
    }

    void MotorControl::setTorque(double torque)
    {
        double  result  = torque * m_direction * m_torque_scalar;
        uint8_t temp_id = m_can | (motor_id + 1);
        m_driver->set_motor_torque(temp_id, result);
    }

    // ------------------------------------------------------------------
    // ------------------------------------------------------------------
    // ------------------------------------------------------------------

    double MotorControl::getPositionRad() const
    {
        uint8_t temp_id = m_can | (motor_id + 1);
        m_driver->get_motor_state(temp_id);
    }

    double MotorControl::getVelocityRad() const
    {
        uint8_t temp_id = m_can | (motor_id + 1);
        m_driver->get_motor_state(temp_id);
    }

    double MotorControl::getPositionDeg() const
    {
        uint8_t temp_id = m_can | (motor_id + 1);
        m_driver->get_motor_state(temp_id);
    }

    double MotorControl::getVelocityDeg() const
    {
        uint8_t temp_id = m_can | (motor_id + 1);
        m_driver->get_motor_state(temp_id);
    }

    double MotorControl::getTorque() const
    {
        uint8_t temp_id = m_can | (motor_id + 1);
        m_driver->get_motor_state(temp_id);
    }
} // namespace pi
