#pragma once

#include <unordered_map>

#include "bitmap.h"
#include "shape.h"

namespace efficient_ransac {

class Cylinder final : public Shape {
 public:
  Cylinder(std::vector<pcl::PointNormal> candidate_points);

  bool isValid(std::vector<pcl::PointNormal> candidate_points,
               Thresholds thresholds) override;

  void computeInliersIndices(
      const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud,
      const Thresholds thresholds,
      const std::vector<bool> &remaining_points) override;

 private:
  inline bool distanceCheck(pcl::PointNormal point, double threshold) override {
    // project the point onto the plane orthogonal to the axis
    Eigen::Vector3f p = point.getVector3fMap();
    Eigen::Vector3f proj_p = p - p.dot(axis_) * axis_;
    return std::abs((proj_p - center_).norm() - radius_) < threshold;
  }
  inline bool normalCheck(pcl::PointNormal point, double threshold) override {
    // project the point onto the plane orthogonal to the axis
    Eigen::Vector3f p = point.getVector3fMap();
    Eigen::Vector3f proj_p = p - p.dot(axis_) * axis_;
    // normal of the cylinder at the projected point
    Eigen::Vector3f normal_cylinder = (proj_p - center_).normalized();
    // normal of the point
    Eigen::Vector3f normal_point = point.getNormalVector3fMap().normalized();
    return std::acos(std::abs(normal_cylinder.dot(normal_point))) < threshold;
  }
  void extractLargestConnectedComponent(
      const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud,
      const CellSize &cell_size) override;

  // cylinder parameters
  Eigen::Vector3f axis_;
  Eigen::Vector3f center_;
  float radius_;
};

}  // namespace efficient_ransac
