#pragma once

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // include OpenCV header file

#include <string>
#include <function>

namespace Rocket
{
    // a simple driver to get depth image and motion data
    class RealsenseDriver
    {
    public:
        RealsenseDriver(const std::string& d435_serial, const std::string t265_serial);

        // TODO : multi-thread
        void registerDepthCallback(std::function<void()> func);
        void registerMotionCallback(std::function<void()> func);

    private:
        std::string m_d435_serial;
        std::string m_t265_serial;

        std::function<void()> m_depth_callback;
        std::function<void()> m_motion_callback;

        rs2::pipeline m_d435_pipe;
        rs2::config   m_d435_cfg;
        rs2::pipeline m_t265_pipe;
        rs2::config   m_t265_cfg;
    };
} // namespace Rocket
