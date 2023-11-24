#pragma once

namespace pi
{
    // single motor control
    class MotorControl
    {
    public:
        MotorControl()  = default;
        ~MotorControl() = default;

        void setPosition();
        void setVelocity();
        void setTorque();

    private:
        double m_direction = 1.0;
        double m_offset    = 0.0;
    };
} // namespace pi
