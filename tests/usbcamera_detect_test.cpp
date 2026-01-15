#include <fmt/core.h>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{name n         |        video0          | USB相机设备名称（如video0）}"
  "{@config-path   | configs/demo.yaml     | yaml配置文件路径}"
  "{d display      |                        | 显示视频流和检测结果}";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  tools::Exiter exiter;

  auto config_path = cli.get<std::string>(0);
  auto device_name = cli.get<std::string>("name");
  auto display = cli.has("display");

  // 初始化USB相机
  io::USBCamera usbcam(device_name, config_path);
  
  auto_aim::Detector detector(config_path, display);
  detector.set_threshold(250.0);
  auto_aim::Solver solver(config_path);
  Eigen::Quaterniond default_q(1, 0, 0, 0);
  solver.set_R_gimbal2world(default_q);

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  int frame_count = 0;
  
  while (!exiter.exit()) {
    usbcam.read(img, timestamp);
    
    if (img.empty()) {
      std::this_thread::sleep_for(10ms);
      continue;
    }

    auto armors = detector.detect(img, frame_count);
    for (auto & armor : armors) {
      solver.solve(armor);

      constexpr double FX = 1320.5372;
      constexpr double FY = 1321.02773;
      constexpr double CX = 633.681963;
      constexpr double CY = 414.417885;

      const double du = armor.center.x - CX;
      const double dv = armor.center.y - CY;
      const double xn = du / FX;
      const double yn = dv / FY;
      const double yaw_rad = std::atan2(xn, 1.0);
      const double pitch_rad = -std::atan2(yn, 1.0);
      const double yaw_deg = yaw_rad * 180.0 / CV_PI;
      const double pitch_deg = pitch_rad * 180.0 / CV_PI;

      double dist = std::sqrt(
        armor.xyz_in_world[0]*armor.xyz_in_world[0] + 
        armor.xyz_in_world[1]*armor.xyz_in_world[1] + 
        armor.xyz_in_world[2]*armor.xyz_in_world[2]);
      tools::logger()->info(
        "装甲板 #{}: 世界坐标({:.3f}, {:.3f}, {:.3f}), 距离={:.3f}m, yaw={:.2f}deg, pitch={:.2f}deg",
        frame_count,
        armor.xyz_in_world[0], armor.xyz_in_world[1], armor.xyz_in_world[2],
        dist, yaw_deg, pitch_deg);
    }
    
    if (display) {
      cv::Mat display_img = img.clone();
      for (const auto & armor : armors) {
        tools::draw_points(display_img, armor.points, {0, 255, 0}, 2);
        double distance = std::sqrt(
          armor.xyz_in_world[0]*armor.xyz_in_world[0] + 
          armor.xyz_in_world[1]*armor.xyz_in_world[1] + 
          armor.xyz_in_world[2]*armor.xyz_in_world[2]);
        std::string info = fmt::format("dist: {:.2f}m", distance);
        tools::draw_text(display_img, info, armor.center, {0, 255, 0});
      }
      cv::imshow("Detection Result", display_img);
      if (cv::waitKey(1) == 'q') break;
    }

    frame_count++;
  }
  
  return 0;
}