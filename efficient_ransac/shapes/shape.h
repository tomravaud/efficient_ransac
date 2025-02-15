#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace efficient_ransac {

struct thresholds {
  double distance = 0.0;
  double normal = 0.0;
};

class Shape {
 public:
  Shape(std::vector<pcl::PointNormal> candidate_points) {}

  virtual bool isValid(std::vector<pcl::PointNormal> candidate_points,
                       thresholds thresholds) = 0;

  virtual void computeInliersIndices(
      const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud,
      const thresholds thresholds) = 0;
  std::vector<int> inliersIndices() { return inliers_indices_; }
  protected:
      std::vector<int> inliers_indices_;
};

}  // namespace efficient_ransac
