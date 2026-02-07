#include <fmt/core.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <thread>

#include "io/usbcamera/usbcamera.hpp"
#include "tools/exiter.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                          | 输出命令行参数说明}"
  "{config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{name n         |        video0          | USB相机设备名称（如video0）}"
  "{@output-folder | assets/img_with_q      | 输出文件夹路径   }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  tools::Exiter exiter;

  auto config_path = cli.get<std::string>("config-path");
  auto device_name = cli.get<std::string>("name");
  auto output_folder = cli.get<std::string>(0);

  auto yaml = YAML::LoadFile(config_path);
  auto pattern_cols = yaml["pattern_cols"].as<int>();
  auto pattern_rows = yaml["pattern_rows"].as<int>();
  auto pattern_type = yaml["pattern_type"].as<std::string>();
  cv::Size pattern_size(pattern_cols, pattern_rows);

  fmt::print("标定板类型: {}\n", pattern_type);
  fmt::print("标定板尺寸: {} x {}\n", pattern_cols, pattern_rows);
  fmt::print("输出文件夹: {}\n", output_folder);
  fmt::print("相机设备: {}\n", device_name);
  fmt::print("\n操作说明:\n");
  fmt::print("  按 's' 键: 保存当前帧为标定图片\n");
  fmt::print("  按 'q' 键: 退出程序\n");
  fmt::print("  按 'c' 键: 清空输出文件夹中的所有图片\n");
  fmt::print("\n");

  io::USBCamera usbcam(device_name, config_path);
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  int image_count = 0;

  while (!exiter.exit()) {
    usbcam.read(img, timestamp);

    if (img.empty()) {
      std::this_thread::sleep_for(10ms);
      continue;
    }

    cv::Mat display_img = img.clone();

    std::vector<cv::Point2f> centers_2d;
    bool success = false;
    if (pattern_type == "circles") {
      success = cv::findCirclesGrid(img, pattern_size, centers_2d, cv::CALIB_CB_SYMMETRIC_GRID);
    } else {
      success = cv::findChessboardCorners(img, pattern_size, centers_2d);
    }

    cv::drawChessboardCorners(display_img, pattern_size, centers_2d, success);

    std::string status_text = success ? "DETECTED" : "NOT DETECTED";
    cv::Scalar status_color = success ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    cv::putText(display_img, status_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, status_color, 2);

    std::string count_text = fmt::format("Images: {}", image_count);
    cv::putText(display_img, count_text, cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);

    cv::imshow("Camera Calibration Capture", display_img);

    int key = cv::waitKey(1) & 0xFF;
    if (key == 'q' || key == 27) {
      break;
    } else if (key == 's') {
      if (success) {
        image_count++;
        auto save_path = fmt::format("{}/{}.jpg", output_folder, image_count);
        cv::imwrite(save_path, img);
        fmt::print("保存图片: {}\n", save_path);
      } else {
        fmt::print("未检测到标定板，无法保存\n");
      }
    } else if (key == 'c') {
      fmt::print("清空输出文件夹...\n");
      for (int i = 1; i <= image_count; i++) {
        auto file_path = fmt::format("{}/{}.jpg", output_folder, i);
        std::remove(file_path.c_str());
      }
      image_count = 0;
      fmt::print("已清空所有图片\n");
    }
  }

  cv::destroyAllWindows();
  fmt::print("\n总共保存了 {} 张标定图片\n", image_count);
  fmt::print("现在可以运行标定程序:\n");
  fmt::print("  ./build/calibrate_camera {}\n", output_folder);

  return 0;
}
