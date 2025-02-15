#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "../utils/pcl_operations.h"

namespace efficient_ransac {

struct thresholds {
  double distance = 0.0;
  double normal = 0.0;
};

class Shape {
 public:
  Shape(std::vector<pcl::PointXYZ> candidate_points,
        std::vector<pcl::Normal> candidate_normals) {}

  virtual bool isValid(std::vector<pcl::PointXYZ> candidate_points,
                       std::vector<pcl::Normal> candidate_normals,
                       thresholds thresholds) = 0;

  virtual std::vector<int> inliersIndices(
      const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
      const std::shared_ptr<pcl::PointCloud<pcl::Normal>> &normals,
      const thresholds thresholds) = 0;
};

}  // namespace efficient_ransac
