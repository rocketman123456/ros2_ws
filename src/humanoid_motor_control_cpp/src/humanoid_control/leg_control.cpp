#include "humanoid_control/leg_control.h"

namespace pi
{
    void initialize(std::shared_ptr<SpiDriver> driver, const std::vector<motor_init_info_t>& infos);
    void reset();

    void positionControlDeg(const std::vector<double>& angles);
    void positionControlRad(const std::vector<double>& angles);
    void torqueControl(const std::vector<double>& torques);

    void poistionControl(const Eigen::Matrix4d& target);
    void forceControl(const Eigen::VectorXd& force);
}
