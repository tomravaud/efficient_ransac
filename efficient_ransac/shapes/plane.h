#pragma once

#include <unordered_map>

#include "bitmap.h"
#include "shape.h"

namespace efficient_ransac {

class Plane final : public Shape {
 public:
  Plane(std::vector<pcl::PointNormal> candidate_points, Thresholds thresholds,
        CellSize cell_size);

  bool isValid(std::vector<pcl::PointNormal> candidate_points) override;

  void computeInliersIndices(
      const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud,
      const std::vector<bool> &remaining_points) override;

 private:
  inline bool distanceCheck(pcl::PointNormal point) override {
    return std::abs(normal_.dot(point.getVector3fMap() - point_)) <
           thresholds_.distance;
  }
  inline bool normalCheck(pcl::PointNormal point) override {
    return acos(std::abs(
               normal_.dot(point.getNormalVector3fMap().normalized()))) <
           thresholds_.normal;
  }
  void extractLargestConnectedComponent(
      const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud) override;

  // plane parameters
  Eigen::Vector3f point_;
  Eigen::Vector3f normal_;
};

}  // namespace efficient_ransac
