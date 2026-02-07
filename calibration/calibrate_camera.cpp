#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <opencv2/opencv.hpp>

#include "tools/img_tools.hpp"

const std::string keys =
  "{help h usage ? |                          | 输出命令行参数说明}"
  "{config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{@input-folder  | assets/img_with_q        | 输入文件夹路径   }";

std::vector<cv::Point3f> centers_3d(const cv::Size & pattern_size, const float center_distance_mm)
{
  std::vector<cv::Point3f> centers_3d;

  for (int i = 0; i < pattern_size.height; i++)
    for (int j = 0; j < pattern_size.width; j++)
      centers_3d.push_back({j * center_distance_mm, i * center_distance_mm, 0});

  return centers_3d;
}

void load(
  const std::string & input_folder, const std::string & config_path, cv::Size & img_size,
  std::vector<std::vector<cv::Point3f>> & obj_points,
  std::vector<std::vector<cv::Point2f>> & img_points)
{
  auto yaml = YAML::LoadFile(config_path);
  auto pattern_cols = yaml["pattern_cols"].as<int>();
  auto pattern_rows = yaml["pattern_rows"].as<int>();
  auto center_distance_mm = yaml["center_distance_mm"].as<double>();
  auto pattern_type = yaml["pattern_type"].as<std::string>();
  cv::Size pattern_size(pattern_cols, pattern_rows);

  for (int i = 1; true; i++) {
    auto img_path = fmt::format("{}/{}.jpg", input_folder, i);
    auto img = cv::imread(img_path);
    if (img.empty()) break;

    img_size = img.size();

    std::vector<cv::Point2f> centers_2d;
    auto success = false;
    if (pattern_type == "circles") {
      success = cv::findCirclesGrid(img, pattern_size, centers_2d, cv::CALIB_CB_SYMMETRIC_GRID);
    } else {
      success = cv::findChessboardCorners(img, pattern_size, centers_2d);
    }

    auto drawing = img.clone();
    cv::drawChessboardCorners(drawing, pattern_size, centers_2d, success);
    cv::resize(drawing, drawing, {}, 0.5, 0.5);
    cv::imshow("Press any to continue", drawing);
    cv::waitKey(0);

    fmt::print("[{}] {}\n", success ? "success" : "failure", img_path);
    if (!success) continue;

    img_points.emplace_back(centers_2d);
    obj_points.emplace_back(centers_3d(pattern_size, center_distance_mm));
  }
}

void print_yaml(const cv::Mat & camera_matrix, const cv::Mat & distort_coeffs, double error)
{
  YAML::Emitter result;
  std::vector<double> camera_matrix_data(
    camera_matrix.begin<double>(), camera_matrix.end<double>());
  std::vector<double> distort_coeffs_data(
    distort_coeffs.begin<double>(), distort_coeffs.end<double>());

  result << YAML::BeginMap;
  result << YAML::Comment(fmt::format("重投影误差: {:.4f}px", error));
  result << YAML::Key << "camera_matrix";
  result << YAML::Value << YAML::Flow << camera_matrix_data;
  result << YAML::Key << "distort_coeffs";
  result << YAML::Value << YAML::Flow << distort_coeffs_data;
  result << YAML::Newline;
  result << YAML::EndMap;

  fmt::print("\n{}\n", result.c_str());
}

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto input_folder = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");

  cv::Size img_size;
  std::vector<std::vector<cv::Point3f>> obj_points;
  std::vector<std::vector<cv::Point2f>> img_points;
  load(input_folder, config_path, img_size, obj_points, img_points);

  cv::Mat camera_matrix, distort_coeffs;
  std::vector<cv::Mat> rvecs, tvecs;
  auto criteria = cv::TermCriteria(
    cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100,
    DBL_EPSILON);
  cv::calibrateCamera(
    obj_points, img_points, img_size, camera_matrix, distort_coeffs, rvecs, tvecs, cv::CALIB_FIX_K3,
    criteria);

  double error_sum = 0;
  size_t total_points = 0;
  for (size_t i = 0; i < obj_points.size(); i++) {
    std::vector<cv::Point2f> reprojected_points;
    cv::projectPoints(
      obj_points[i], rvecs[i], tvecs[i], camera_matrix, distort_coeffs, reprojected_points);

    total_points += reprojected_points.size();
    for (size_t j = 0; j < reprojected_points.size(); j++)
      error_sum += cv::norm(img_points[i][j] - reprojected_points[j]);
  }
  auto error = error_sum / total_points;

  print_yaml(camera_matrix, distort_coeffs, error);
}
