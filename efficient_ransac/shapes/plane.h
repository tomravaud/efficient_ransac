#pragma once

#include <unordered_map>

#include "bitmap.h"
#include "shape.h"

namespace efficient_ransac {

class Plane final : public Shape {
 public:
  Plane(std::vector<pcl::PointNormal> candidate_points);

  bool isValid(std::vector<pcl::PointNormal> candidate_points,
               Thresholds thresholds) override;

  void computeInliersIndices(
      const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud,
      const Thresholds thresholds,
      const std::vector<bool> &remaining_points) override;

 private:
  inline bool distanceCheck(pcl::PointNormal point, double threshold) override {
    return std::abs(normal_.dot(point.getVector3fMap() - point_)) < threshold;
  }
  inline bool normalCheck(pcl::PointNormal point, double threshold) override {
    return acos(std::abs(normal_.dot(
               point.getNormalVector3fMap().normalized()))) < threshold;
  }
  void extractLargestConnectedComponent(
      const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud,
      const CellSize &cell_size) override;

  // plane parameters
  Eigen::Vector3f point_;
  Eigen::Vector3f normal_;
};

}  // namespace efficient_ransac
