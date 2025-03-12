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

  // void computeInliersIndices(
  //     const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud,
  //     const std::vector<bool> &remaining_points,
  //     const bool compute_connected_components
  //   ) override;

 private:
  inline float distance(pcl::PointNormal point) override {
    return std::abs(normal_.dot(point.getVector3fMap() - point_));
  }
  inline float angle(pcl::PointNormal point) override {
    return acos(std::abs(
               normal_.dot(point.getNormalVector3fMap().normalized())));
  }
  void extractLargestConnectedComponent(
      const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud) override;

  // plane parameters
  Eigen::Vector3f point_;
  Eigen::Vector3f normal_;
};

}  // namespace efficient_ransac
