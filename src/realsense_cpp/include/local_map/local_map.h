#pragma once

#include <Eigen/Eigen>

#include <shared_mutex>
#include <vector>

namespace Rocket
{
    struct LocalMapConfig
    {
        float  m_grid_size_x {0};
        float  m_grid_size_y {0};
        size_t m_grid_count_front {0};
        size_t m_grid_count_back {0};
        size_t m_grid_count_left {0};
        size_t m_grid_count_right {0};
    };

    struct LocalMapData
    {
        std::vector<Eigen::Vector3f> height_map; // position in local grid map coordinate
    };

    class LocalMapBuilder
    {
    public:
        LocalMapBuilder(const LocalMapConfig& config, float alpha = 0.1);

        void initialize(float* points, size_t count);
        void initialize(const std::vector<Eigen::Vector3f>& points);

        void setNewPose(const Eigen::Quaternionf& q, const Eigen::Vector3f& pos);
        void setNewPoints(float* points, size_t count);
        void setNewPoints(const std::vector<Eigen::Vector3f>& points);

    private:
        void generateLocalMap();
        void blendPrevLocalMap();

        // utils
        Eigen::Matrix4f projectPoseToGround(const Eigen::Matrix4f& pose);
        bool            projectGridIndexCalculate(double x, double y, size_t* nx, size_t* ny);
        bool            projectGridIndexCalculate(const Eigen::Vector3f& pos, size_t* nx, size_t* ny);

    private:
        LocalMapConfig m_grid_config;
        LocalMapData   m_grid_data_curr;
        LocalMapData   m_grid_data_prev;
        float          m_grid_x1; // left bottom corner
        float          m_grid_y1; // left bottom corner
        float          m_grid_x2; // right top corner
        float          m_grid_y2; // right top corner

        float m_alpha;
        float m_initial_height;

        // std::shared_mutex m_grid_data_mutex;
        // std::lock_guard<std::mutex> guard(m_grid_data_mutex);

        Eigen::Matrix4f    m_map_pose_curr;
        Eigen::Matrix4f    m_map_pose_prev;
        Eigen::Matrix4f    m_pose_curr;
        Eigen::Matrix4f    m_pose_prev;
        Eigen::Quaternionf m_q_curr;
        Eigen::Quaternionf m_q_prev;
        Eigen::Vector3f    m_pos_curr;
        Eigen::Vector3f    m_pos_prev;
    };
} // namespace Rocket
