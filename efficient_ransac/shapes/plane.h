#pragma once

#include "shape.h"

namespace efficient_ransac {

class Plane : public Shape {
 public:
  Plane(std::vector<pcl::PointXYZ> candidate_points,
        std::vector<pcl::Normal> candidate_normals);

  bool isValid(std::vector<pcl::PointXYZ> candidate_points,
               std::vector<pcl::Normal> candidate_normals,
               thresholds thresholds) override;

  std::vector<int> inliersIndices(
      const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
      const std::shared_ptr<pcl::PointCloud<pcl::Normal>> &normals,
      const thresholds thresholds) override;

 private:
  inline bool distanceCheck(pcl::PointXYZ point, double threshold) {
    return abs(dot(normal_, point - point_)) < threshold;
  }
  inline bool normalCheck(pcl::Normal normal, double threshold) {
    return acos(abs(dot(normal_, normal))) < threshold;
  }

  pcl::PointXYZ point_;
  pcl::Normal normal_;
};

}  // namespace efficient_ransac
