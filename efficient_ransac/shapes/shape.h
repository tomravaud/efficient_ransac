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
  Shape(std::vector<pcl::PointNormal> candidate_points) {}

  virtual bool isValid(std::vector<pcl::PointNormal> candidate_points,
                       Thresholds thresholds) = 0;

  virtual void computeInliersIndices(
      const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud,
      const Thresholds thresholds,
      const std::vector<bool> &remaining_points) = 0;

  // Getters
  std::vector<int> inliers_indices() { return inliers_indices_; }

 protected:
  virtual inline bool distanceCheck(pcl::PointNormal point,
                                    double threshold) = 0;
  virtual inline bool normalCheck(pcl::PointNormal normal,
                                  double threshold) = 0;

  virtual void extractLargestConnectedComponent(
      const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud,
      const CellSize &cell_size) = 0;

  std::vector<int> inliers_indices_;
};

}  // namespace efficient_ransac
