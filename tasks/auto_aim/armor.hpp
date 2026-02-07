#ifndef AUTO_AIM__ARMOR_HPP
#define AUTO_AIM__ARMOR_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace auto_aim
{
enum Color
{
  red,
  blue,
  extinguish,
  purple
};
const std::vector<std::string> COLORS = {"red", "blue", "extinguish", "purple"};

enum ArmorType
{
  big,
  small
};
const std::vector<std::string> ARMOR_TYPES = {"big", "small"};

enum ArmorName
{
  one,
  two,
  three,
  four,
  five,
  sentry,
  outpost,
  base,
  not_armor
};
const std::vector<std::string> ARMOR_NAMES = {"one",    "two",     "three", "four",     "five",
                                              "sentry", "outpost", "base",  "not_armor"};

struct Lightbar
{
  std::size_t id;
  Color color;
  cv::Point2f center, top, bottom, top2bottom;
  std::vector<cv::Point2f> points;
  double angle, angle_error, length, width, ratio;
  cv::RotatedRect rotated_rect;

  Lightbar(const cv::RotatedRect & rotated_rect, std::size_t id);
  Lightbar() {};
};

struct Armor
{
  Color color;
  Lightbar left, right;
  cv::Point2f center;
  cv::Point2f center_norm;
  std::vector<cv::Point2f> points;

  double ratio;
  double side_ratio;
  double rectangular_error;

  ArmorType type;
  ArmorName name;
  int class_id;
  cv::Rect box;
  double confidence;
  bool duplicated;

  Eigen::Vector3d xyz_in_gimbal;  // 单位：m
  Eigen::Vector3d ypr_in_gimbal;  // 单位：rad
  Eigen::Vector3d ypd_in_gimbal;   // 球坐标系

  double yaw_raw;  // rad

  Armor(const Lightbar & left, const Lightbar & right);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__ARMOR_HPP
