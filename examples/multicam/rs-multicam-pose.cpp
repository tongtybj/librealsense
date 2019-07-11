// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
#include "example.hpp"              // Include short list of convenience functions for rendering
#include <chrono>
#include <map>
#include <vector>

int main(int argc, char * argv[]) try
{
    // Create a simple OpenGL window for rendering:
  //window app(1280, 960, "CPP Multi-Camera Example");

    rs2::context                ctx;            // Create librealsense context for managing devices

    rs2::colorizer              colorizer;      // Utility class to convert depth data RGB colorspace

    std::vector<rs2::pipeline>  pipelines;
    std::vector<std::string>  id_list;

    // Start a streaming pipe per each connected device
    for (auto&& dev : ctx.query_devices())
      {
        rs2::pipeline pipe(ctx);
        rs2::config cfg;
        cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        cfg.disable_all_streams();
        cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);

        /* we can only enable fhisheye bot image */
        //cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
        //cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);

        std::cout << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
        std::cout << dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID) << std::endl;
        pipe.start(cfg);
        std::cout << "ok" << std::endl;
        pipelines.emplace_back(pipe);
        id_list.emplace_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    }

    std::vector<uint64_t> pose_counter(pipelines.size());
    std::vector<uint64_t> frame_counter(pipelines.size());

    std::vector<std::chrono::_V2::system_clock::time_point> last_print(2, std::chrono::system_clock::now());
    // We'll keep track for the last frame of each stream available to make the presentation persistent
    std::map<int, rs2::frame> render_frames;

    // Main app loop
    while (true)
    //while (app)
    {
        // Collect the new frames from all the connected devices
        std::vector<rs2::frame> new_frames;
        int i = 0;
        for (auto &&pipe : pipelines)
          {
            rs2::frameset fs;
            if (pipe.poll_for_frames(&fs))
              {

                auto f = fs.first_or_default(RS2_STREAM_POSE);
                auto pose_data = f.as<rs2::pose_frame>().get_pose_data();
                //std::cout << "the size of frameset: " << fs.size() << std::endl;
                //std::cout << "Device Position of " << pipe.get_active_profile().get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << "at " << f.get_timestamp()  << ": " << std::fixed << pose_data.translation.x << " " << pose_data.translation.y << " " << pose_data.translation.z << " (meters)" << std::endl;

                for (const rs2::frame& f : fs)
                  new_frames.emplace_back(f);

                if (auto fp = f.as<rs2::pose_frame>())
                  pose_counter.at(i)++;

                else if (auto fs = f.as<rs2::frameset>())
                  frame_counter.at(i)++;

              }

            // Convert the newly-arrived frames to render-firendly format
#if 0
            for (const auto& frame : new_frames)
              {
                render_frames[frame.get_profile().unique_id()] = colorizer.process(frame);
              }

            // Present all the collected frames with openGl mosaic
            //app.show(render_frames);
#endif

#if 1
            auto now = std::chrono::system_clock::now();
            if (now - last_print.at(i) >= std::chrono::seconds(1)) {
              std::cout << pipe.get_active_profile().get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)
                        << ": Pose rate: "  << pose_counter.at(i) << " "
                        << "Image rate: " << frame_counter.at(i) << std::endl;
              pose_counter.at(i) = 0;
              frame_counter.at(i) = 0;
              last_print.at(i) = now;
            }
#endif
            i++;
        }
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
