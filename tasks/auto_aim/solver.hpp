#ifndef AUTO_AIM__SOLVER_HPP
#define AUTO_AIM__SOLVER_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

#include "armor.hpp"

namespace auto_aim
{
class Solver
{
public:
  explicit Solver(const std::string & config_path);

  void solve(Armor & armor) const;

  std::vector<cv::Point2f> reproject_armor(
    const Eigen::Vector3d & xyz_in_gimbal, double yaw, ArmorType type, ArmorName name) const;

  double outpost_reprojection_error(Armor armor, const double & pitch);

private:
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;

  void optimize_yaw(Armor & armor) const;

  double armor_reprojection_error(const Armor & armor, double yaw, const double & inclined) const;
  double SJTU_cost(
    const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
    const double & inclined) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__SOLVER_HPP
