// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <iomanip>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <string>
#include <vector>

// typedef struct rs2_pose
// {
//     rs2_vector      translation;          /**< X, Y, Z values of translation, in meters (relative to initial position)                                    */
//     rs2_vector      velocity;             /**< X, Y, Z values of velocity, in meters/sec                                                                  */
//     rs2_vector      acceleration;         /**< X, Y, Z values of acceleration, in meters/sec^2                                                            */
//     rs2_quaternion  rotation;             /**< Qi, Qj, Qk, Qr components of rotation as represented in quaternion rotation (relative to initial position) */
//     rs2_vector      angular_velocity;     /**< X, Y, Z values of angular velocity, in radians/sec                                                         */
//     rs2_vector      angular_acceleration; /**< X, Y, Z values of angular acceleration, in radians/sec^2                                                   */
//     unsigned int    tracker_confidence;   /**< Pose confidence 0x0 - Failed, 0x1 - Low, 0x2 - Medium, 0x3 - High                                          */
//     unsigned int    mapper_confidence;    /**< Pose map confidence 0x0 - Failed, 0x1 - Low, 0x2 - Medium, 0x3 - High                                      */
// } rs2_pose;

int main(int argc, char* argv[])
try
{
    std::string serial = "952322110641";

    rs2::context ctx; // Create librealsense context for managing devices

    std::string   t265_serial = "952322110641";
    std::string   d435_serial = "939622071659";
    rs2::pipeline t265_pipe(ctx);
    rs2::pipeline d435_pipe(ctx);
    rs2::config   t265_cfg;
    rs2::config   d435_cfg;

    t265_cfg.enable_device(t265_serial);
    t265_cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF, 200);

    d435_cfg.enable_device(d435_serial);
    d435_cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);
    d435_cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 60);
    // d435_cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 60);

    // Start pipeline with chosen configuration
    auto t265_profile = t265_pipe.start(t265_cfg);
    auto t265_device  = t265_profile.get_device();
    std::cout << "t265 serial: " << t265_device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;

    auto d435_profile = d435_pipe.start(d435_cfg);
    auto d435_device  = d435_profile.get_device();
    std::cout << "d435 serial: " << d435_device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;

    // Define two align objects. One will be used to align to depth viewport and the other to color.
    // Creating align object is an expensive operation that should not be performed in the main loop
    rs2::align align_to_depth(RS2_STREAM_DEPTH);
    rs2::align align_to_color(RS2_STREAM_COLOR);

    // Declare filters
    rs2::colorizer           color_map;                // Declare depth colorizer for pretty visualization of depth data
    rs2::rates_printer       printer;                  // Declare rates printer for showing streaming rates of the enabled streams.
    rs2::decimation_filter   dec_filter;               // Decimation - reduces depth frame density
    rs2::threshold_filter    thr_filter(0.15, 1.5);    // Threshold  - removes values outside recommended range
    rs2::spatial_filter      spat_filter;              // Spatial    - edge-preserving spatial smoothing
    rs2::temporal_filter     temp_filter;              // Temporal   - reduces temporal noise
    rs2::disparity_transform depth_to_disparity(true); // Declare disparity transform from depth to disparity and vice versa
    rs2::disparity_transform disparity_to_depth(false);

    rs2::pointcloud pc;     // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::points     points; // We want the points object to be persistent so we can display the last cloud when a frame drops

    // Main loop
    while (true)
    {
        {
            // Wait for the next set of frames from the camera
            auto frames = t265_pipe.wait_for_frames();
            // Get a frame from the pose stream
            auto f = frames.first_or_default(RS2_STREAM_POSE);
            // Cast the frame to pose_frame and get its data
            auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

            // Print the x, y, z values of the translation, relative to initial position
            std::cout << "\r"
                      << "Device Position: " << std::setprecision(3) << std::fixed << pose_data.translation.x << " " << pose_data.translation.y << " "
                      << pose_data.translation.z << " (meters)";
        }

        {

            // Wait for the next set of frames from the camera
            // Block program until frames arrive
            frames = d435_pipe.wait_for_frames();
            frames = align_to_color.process(frames);

            // Try to get a frame of a depth image
            rs2::frame color_frame = frames.get_color_frame();
            rs2::frame ir_frame    = frames.first(RS2_STREAM_INFRARED);
            rs2::frame depth_frame = frames.get_depth_frame();

            // auto filtered = dec_filter.process(depth_frame);
            auto filtered_frame = thr_filter.process(depth_frame);
            filtered_frame      = spat_filter.process(filtered_frame);
            filtered_frame      = temp_filter.process(filtered_frame);

            rs2::video_frame colorized_depth = color_map.process(filtered_frame);

            // Creating OpenCV Matrix from a color image
            // cv::Mat color(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            // // cv::Mat depth(cv::Size(640, 480), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
            // cv::Mat depth(cv::Size(640, 480), CV_8UC3, (void*)colorized_depth.get_data(), cv::Mat::AUTO_STEP);
            // cv::Mat ir(cv::Size(640, 480), CV_8UC1, (void*)ir_frame.get_data(), cv::Mat::AUTO_STEP);

            pc.map_to(color_frame);                               // Tell pointcloud object to map to this color frame
            points            = pc.calculate(filtered_frame);     // Generate the pointcloud and texture mappings
            auto   vertices   = points.get_vertices();            // get vertices
            auto   tex_coords = points.get_texture_coordinates(); // and texture coordinates
            float* points_ptr = (float*)(vertices);
            for (int i = 0; i < points.size(); i++)
            {
                if (vertices[i].z)
                {
                    // TODO
                }
            }

            // equalizeHist(ir, ir);
            // applyColorMap(ir, ir, cv::COLORMAP_JET);

            // equalizeHist(depth, depth);
            // applyColorMap(depth, depth, cv::COLORMAP_JET);

            // imshow("Depth", depth);
            // imshow("Color", color);
            // imshow("IR", ir);
            // if (cv::waitKey(1) == 27)
            //     break;
        }
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error& e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
