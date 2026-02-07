#include "detector.hpp"

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <numeric>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace auto_aim
{
Detector::Detector(const std::string & config_path, bool debug)
: debug_(debug)
{
  auto yaml = YAML::LoadFile(config_path);

  // 读取敌方颜色
  std::string enemy_color_str = yaml["enemy_color"].as<std::string>();
  if (enemy_color_str == "red") {
    enemy_color_ = Color::red;
  } else if (enemy_color_str == "blue") {
    enemy_color_ = Color::blue;
  } else {
    enemy_color_ = Color::blue;  // 默认蓝色
    tools::logger()->warn("未知的敌方颜色: {}, 使用默认蓝色", enemy_color_str);
  }

  threshold_ = yaml["threshold"].as<double>();
  max_angle_error_ = yaml["max_angle_error"].as<double>() / 57.3;
  min_lightbar_ratio_ = yaml["min_lightbar_ratio"].as<double>();
  max_lightbar_ratio_ = yaml["max_lightbar_ratio"].as<double>();
  min_lightbar_length_ = yaml["min_lightbar_length"].as<double>();
  min_armor_ratio_ = yaml["min_armor_ratio"].as<double>();
  max_armor_ratio_ = yaml["max_armor_ratio"].as<double>();
  max_side_ratio_ = yaml["max_side_ratio"].as<double>();
  min_confidence_ = yaml["min_confidence"].as<double>();
  max_rectangular_error_ = yaml["max_rectangular_error"].as<double>() / 57.3;

  save_path_ = "patterns";
  std::filesystem::create_directory(save_path_);
}

std::list<Armor> Detector::detect(const cv::Mat & bgr_img, int frame_count)
{
  cv::Mat gray_img;
  cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);
  cv::Mat binary_img;
  cv::threshold(gray_img, binary_img, threshold_, 255, cv::THRESH_BINARY);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

  std::size_t lightbar_id = 0;
  std::list<Lightbar> lightbars;
  for (const auto & contour : contours) {
    auto rotated_rect = cv::minAreaRect(contour);
    auto lightbar = Lightbar(rotated_rect, lightbar_id);

    if (!check_geometry(lightbar)) continue;

    lightbar.color = get_color(bgr_img, contour);
    
    // 只保留红色和蓝色灯条，过滤其他颜色
    if (lightbar.color != Color::red && lightbar.color != Color::blue) continue;
    
    lightbars.emplace_back(lightbar);
    lightbar_id += 1;
  }

  lightbars.sort([](const Lightbar & a, const Lightbar & b) { return a.center.x < b.center.x; });

  std::list<Armor> armors;
  for (auto left = lightbars.begin(); left != lightbars.end(); left++) {
    for (auto right = std::next(left); right != lightbars.end(); right++) {
      if (left->color != right->color) continue;
      
      // 过滤非敌方颜色的装甲板
      if (left->color != enemy_color_) continue;

      auto armor = Armor(*left, *right);
      if (!check_geometry(armor)) continue;

      armor.center_norm = get_center_norm(bgr_img, armor.center);
      armors.emplace_back(armor);
    }
  }

  // 检查装甲板是否存在共用灯条的情况
  for (auto armor1 = armors.begin(); armor1 != armors.end(); armor1++) {
    for (auto armor2 = std::next(armor1); armor2 != armors.end(); armor2++) {
      if (
        armor1->left.id != armor2->left.id && armor1->left.id != armor2->right.id &&
        armor1->right.id != armor2->left.id && armor1->right.id != armor2->right.id) {
        continue;
      }

      if (armor1->left.id == armor2->left.id || armor1->right.id == armor2->right.id) {
        auto area1 = armor1->ratio * std::max(armor1->left.length, armor1->right.length);
        auto area2 = armor2->ratio * std::max(armor2->left.length, armor2->right.length);
        if (area1 < area2)
          armor2->duplicated = true;
        else
          armor1->duplicated = true;
      }

      if (armor1->left.id == armor2->right.id || armor1->right.id == armor2->left.id) {
        if (armor1->rectangular_error > armor2->rectangular_error)
          armor1->duplicated = true;
        else
          armor2->duplicated = true;
      }
    }
  }

  armors.remove_if([&](const Armor & a) { return a.duplicated; });

  if (debug_) show_result(binary_img, bgr_img, lightbars, armors, frame_count);

  return armors;
}

bool Detector::detect(Armor & armor, const cv::Mat & bgr_img)
{
  auto tl = armor.points[0];
  auto tr = armor.points[1];
  auto br = armor.points[2];
  auto bl = armor.points[3];
  auto lt2b = bl - tl;
  auto rt2b = br - tr;
  auto tl1 = (tl + bl) / 2 - lt2b;
  auto bl1 = (tl + bl) / 2 + lt2b;
  auto br1 = (tr + br) / 2 + rt2b;
  auto tr1 = (tr + br) / 2 - rt2b;
  auto tl2tr = tr1 - tl1;
  auto bl2br = br1 - bl1;
  auto tl2 = (tl1 + tr) / 2 - 0.75 * tl2tr;
  auto tr2 = (tl1 + tr) / 2 + 0.75 * tl2tr;
  auto bl2 = (bl1 + br) / 2 - 0.75 * bl2br;
  auto br2 = (bl1 + br) / 2 + 0.75 * bl2br;
  std::vector<cv::Point> points = {tl2, tr2, br2, bl2};
  auto armor_rotaterect = cv::minAreaRect(points);
  cv::Rect boundingBox = armor_rotaterect.boundingRect();
  if (
    boundingBox.x < 0 || boundingBox.y < 0 || boundingBox.x + boundingBox.width > bgr_img.cols ||
    boundingBox.y + boundingBox.height > bgr_img.rows) {
    return false;
  }

  cv::Mat armor_roi = bgr_img(boundingBox);
  if (armor_roi.empty()) {
    return false;
  }

  cv::Mat gray_img;
  cv::cvtColor(armor_roi, gray_img, cv::COLOR_BGR2GRAY);
  cv::Mat binary_img;
  cv::threshold(gray_img, binary_img, threshold_, 255, cv::THRESH_BINARY);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  std::size_t lightbar_id = 0;
  std::list<Lightbar> lightbars;
  for (const auto & contour : contours) {
    auto rotated_rect = cv::minAreaRect(contour);
    auto lightbar = Lightbar(rotated_rect, lightbar_id);

    if (!check_geometry(lightbar)) continue;

    lightbar.color = get_color(bgr_img, contour);
    if (lightbar.color != Color::red && lightbar.color != Color::blue) continue;
    lightbars.emplace_back(lightbar);
    lightbar_id += 1;
  }

  if (lightbars.size() < 2) return false;

  lightbars.sort([](const Lightbar & a, const Lightbar & b) { return a.center.x < b.center.x; });

  Lightbar * closest_left_lightbar = nullptr;
  Lightbar * closest_right_lightbar = nullptr;
  float min_distance_tl_bl = std::numeric_limits<float>::max();
  float min_distance_br_tr = std::numeric_limits<float>::max();
  for (auto & lightbar : lightbars) {
    float distance_tl_bl =
      cv::norm(tl - (lightbar.top + cv::Point2f(boundingBox.x, boundingBox.y))) +
      cv::norm(bl - (lightbar.bottom + cv::Point2f(boundingBox.x, boundingBox.y)));
    if (distance_tl_bl < min_distance_tl_bl) {
      min_distance_tl_bl = distance_tl_bl;
      closest_left_lightbar = &lightbar;
    }
    float distance_br_tr =
      cv::norm(br - (lightbar.bottom + cv::Point2f(boundingBox.x, boundingBox.y))) +
      cv::norm(tr - (lightbar.top + cv::Point2f(boundingBox.x, boundingBox.y)));
    if (distance_br_tr < min_distance_br_tr) {
      min_distance_br_tr = distance_br_tr;
      closest_right_lightbar = &lightbar;
    }
  }



  if (
    closest_left_lightbar && closest_right_lightbar &&
    min_distance_br_tr + min_distance_tl_bl < 15) {
    // 将四个点从armor_roi坐标系转换到原始图像坐标系
    armor.points[0] = closest_left_lightbar->top + cv::Point2f(boundingBox.x, boundingBox.y);
    armor.points[1] = closest_right_lightbar->top + cv::Point2f(boundingBox.x, boundingBox.y);
    armor.points[2] = closest_right_lightbar->bottom + cv::Point2f(boundingBox.x, boundingBox.y);
    armor.points[3] = closest_left_lightbar->bottom + cv::Point2f(boundingBox.x, boundingBox.y);
    return true;
  }

  return false;
}

bool Detector::check_geometry(const Lightbar & lightbar) const
{
  auto angle_ok = lightbar.angle_error < max_angle_error_;
  auto ratio_ok = lightbar.ratio > min_lightbar_ratio_ && lightbar.ratio < max_lightbar_ratio_;
  auto length_ok = lightbar.length > min_lightbar_length_;
  return angle_ok && ratio_ok && length_ok;
}

bool Detector::check_geometry(const Armor & armor) const
{
  auto ratio_ok = armor.ratio > min_armor_ratio_ && armor.ratio < max_armor_ratio_;
  auto side_ratio_ok = armor.side_ratio < max_side_ratio_;
  auto rectangular_error_ok = armor.rectangular_error < max_rectangular_error_;
  return ratio_ok && side_ratio_ok && rectangular_error_ok;
}





Color Detector::get_color(const cv::Mat & bgr_img, const std::vector<cv::Point> & contour) const
{
  cv::Mat hsv_img;
  cv::cvtColor(bgr_img, hsv_img, cv::COLOR_BGR2HSV);

  cv::Scalar red_lower1(0, 50, 50);
  cv::Scalar red_upper1(10, 255, 255);
  cv::Scalar red_lower2(170, 50, 50);
  cv::Scalar red_upper2(180, 255, 255);
  cv::Scalar blue_lower(100, 50, 50);
  cv::Scalar blue_upper(130, 255, 255);

  cv::Mat mask = cv::Mat::zeros(bgr_img.size(), CV_8UC1);
  cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{contour}, cv::Scalar(255));

  int red_count = 0, blue_count = 0;
  int total_count = 0;
  const int step = 2;
  for (int y = 0; y < mask.rows; y += step) {
    for (int x = 0; x < mask.cols; x += step) {
      if (mask.at<uchar>(y, x) > 0) {
        if (x < 0 || x >= hsv_img.cols || y < 0 || y >= hsv_img.rows) {
          continue;
        }
        
        cv::Vec3b hsv = hsv_img.at<cv::Vec3b>(y, x);
        int h = hsv[0];
        int s = hsv[1];
        int v = hsv[2];
        
        total_count++;
        if ((h >= red_lower1[0] && h <= red_upper1[0] && s >= red_lower1[1] && v >= red_lower1[2]) ||
            (h >= red_lower2[0] && h <= red_upper2[0] && s >= red_lower2[1] && v >= red_lower2[2])) {
          red_count++;
        }
        else if (h >= blue_lower[0] && h <= blue_upper[0] && s >= blue_lower[1] && v >= blue_lower[2]) {
          blue_count++;
        }
      }
    }
  }
  
  if (total_count < 10) {
    for (const auto & point : contour) {
      if (point.x < 0 || point.x >= hsv_img.cols || point.y < 0 || point.y >= hsv_img.rows) {
        continue;
      }
      
      cv::Vec3b hsv = hsv_img.at<cv::Vec3b>(point);
      int h = hsv[0];
      int s = hsv[1];
      int v = hsv[2];
      
      total_count++;
      if ((h >= red_lower1[0] && h <= red_upper1[0] && s >= red_lower1[1] && v >= red_lower1[2]) ||
          (h >= red_lower2[0] && h <= red_upper2[0] && s >= red_lower2[1] && v >= red_lower2[2])) {
        red_count++;
      }
      else if (h >= blue_lower[0] && h <= blue_upper[0] && s >= blue_lower[1] && v >= blue_lower[2]) {
        blue_count++;
      }
    }
  }
  
  const double min_color_ratio = 0.3;
  if (total_count > 0) {
    double red_ratio = static_cast<double>(red_count) / total_count;
    double blue_ratio = static_cast<double>(blue_count) / total_count;
    
    if (blue_ratio > red_ratio && blue_ratio >= min_color_ratio) {
      return Color::blue;
    } else if (red_ratio > blue_ratio && red_ratio >= min_color_ratio) {
      return Color::red;
    }
  }
  
  return blue_count > red_count ? Color::blue : Color::red;
}



ArmorType Detector::get_type(const Armor & armor)
{
  if (armor.ratio > 3.0) {
    return ArmorType::big;
  }

  if (armor.ratio < 2.5) {
    return ArmorType::small;
  }

  if (armor.name == ArmorName::one || armor.name == ArmorName::base) {
    return ArmorType::big;
  }
  return ArmorType::small;
}

cv::Point2f Detector::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const
{
  auto h = bgr_img.rows;
  auto w = bgr_img.cols;
  return {center.x / w, center.y / h};
}

void Detector::save(const Armor & armor) const
{
}

void Detector::show_result(
  const cv::Mat & binary_img, const cv::Mat & bgr_img, const std::list<Lightbar> & lightbars,
  const std::list<Armor> & armors, int frame_count) const
{
  auto detection = bgr_img.clone();
  tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});

  for (const auto & lightbar : lightbars) {
    auto info = fmt::format(
      "{:.1f} {:.1f} {:.1f} {}", lightbar.angle_error * 57.3, lightbar.ratio, lightbar.length,
      COLORS[lightbar.color]);
    tools::draw_text(detection, info, lightbar.top, {0, 255, 255});
    tools::draw_points(detection, lightbar.points, {0, 255, 255}, 3);
  }

  for (const auto & armor : armors) {
    auto info = fmt::format(
      "ratio:{:.2f} side:{:.2f} err:{:.1f}deg", armor.ratio, armor.side_ratio,
      armor.rectangular_error * 57.3);
    tools::draw_points(detection, armor.points, {0, 255, 0});
    tools::draw_text(detection, info, armor.left.bottom, {0, 255, 0});
  }

  cv::Mat binary_img2;
  cv::resize(binary_img, binary_img2, {}, 0.5, 0.5);
  cv::resize(detection, detection, {}, 0.5, 0.5);
  cv::imshow("Binary Image", binary_img2);
  cv::imshow("Detection Result", detection);
  cv::waitKey(1);
}



}  // namespace auto_aim