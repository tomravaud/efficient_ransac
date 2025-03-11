#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "bitmap.h"

namespace efficient_ransac {

struct Thresholds {
  float distance = 0.0;
  float normal = 0.0;
};

class Shape {
 public:
  Shape(std::vector<pcl::PointNormal> candidate_points, Thresholds thresholds,
        CellSize cell_size) {
    thresholds_ = thresholds;
    cell_size_ = cell_size;
  };

  virtual bool isValid(std::vector<pcl::PointNormal> candidate_points) = 0;

  virtual void computeInliersIndices(
      const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud,
      const std::vector<bool> &remaining_points) = 0;
  
  void removeFromInliersIndices(std::vector<int> indices_to_remove); 

  // Getters
  std::vector<int> inliers_indices() { return inliers_indices_; }

 protected:
  virtual inline bool distanceCheck(pcl::PointNormal point) = 0;
  virtual inline bool normalCheck(pcl::PointNormal normal) = 0;

  virtual void extractLargestConnectedComponent(
      const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud) = 0;

  std::vector<int> inliers_indices_;

  Thresholds thresholds_;
  CellSize cell_size_;
};

}  // namespace efficient_ransac
