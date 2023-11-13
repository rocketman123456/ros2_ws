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
    };
} // namespace pi
