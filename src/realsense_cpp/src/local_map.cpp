#include "realsense_cpp/local_map.h"

#include <cmath>

namespace Rocket
{
    LocalMapBuilder::LocalMapBuilder(const LocalGridConfig& config, float alpha) {}

    void LocalMapBuilder::initialize(float* points, size_t count) {}

    void LocalMapBuilder::initialize(const std::vector<Eigen::Vector3f>& points) {}

    void LocalMapBuilder::setNewPose(const Eigen::Quaternionf& q, const Eigen::Vector3f& pos) {}

    void LocalMapBuilder::setNewPoints(float* points, size_t count) {}

    void LocalMapBuilder::setNewPoints(const std::vector<Eigen::Vector3f>& points) {}

    Eigen::Matrix4f LocalMapBuilder::projectPoseToGround(const Eigen::Matrix4f& pose)
    {
        Eigen::Matrix<float, 3, 4> pre_scalar;
        pre_scalar << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
        Eigen::Vector4f x_hat;
        x_hat << 1, 0, 0, 0;

        Eigen::Vector3f x_g = pre_scalar * pose * x_hat;
        x_g.normalize();
        Eigen::Vector3f z_g(0, 0, 1);
        Eigen::Vector3f y_g = z_g.cross(x_g);

        Eigen::Matrix4f G   = Eigen::Matrix4f::Zero();
        G.block<3, 1>(0, 0) = x_g;
        G.block<3, 1>(0, 1) = y_g;
        G.block<3, 1>(0, 2) = z_g;
        G(0, 3)             = pose(0, 3);
        G(1, 3)             = pose(1, 3);
        G(3, 3)             = 1;

        return G;
    }

    bool LocalMapBuilder::projectGridIndexCalculate(double x, double y, size_t* nx, size_t* ny)
    {
        if (x < m_grid_x1 && y < m_grid_y1)
            return false;
        else if (x > m_grid_x2 && y > m_grid_y2)
            return false;

        *nx = fmod(x - m_grid_x1, m_grid_config.m_grid_size_x);
        *ny = fmod(y - m_grid_y1, m_grid_config.m_grid_size_y);
        return true;
    }
} // namespace Rocket
