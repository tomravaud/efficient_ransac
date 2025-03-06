#pragma once

#include <unordered_map>

#include "bitmap.h"
#include "shape.h"

namespace efficient_ransac {

class Plane : public Shape {
 public:
  Plane(std::vector<pcl::PointNormal> candidate_points);

  bool isValid(std::vector<pcl::PointNormal> candidate_points,
               thresholds thresholds) override;

  void computeInliersIndices(
      const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud,
      const thresholds thresholds,
      const std::vector<bool> &remaining_points) override;

  void extractLargestConnectedComponent(
      const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud,
      float beta) {
    // local coordinate system in the plane
    Eigen::Vector3f u, v;
    u = normal_.unitOrthogonal();
    v = normal_.cross(u);

    // fill the bitmap (only active cells are stored)
    std::unordered_map<CellCoord, std::vector<int>, CellCoordHasher> bitmap;
    for (int idx : inliers_indices_) {
      Eigen::Vector3f local_coords = cloud->at(idx).getVector3fMap() - point_;
      int cx = static_cast<int>(std::floor(local_coords.dot(u) / beta));
      int cy = static_cast<int>(std::floor(local_coords.dot(v) / beta));
      CellCoord key = {cx, cy};
      bitmap[key].push_back(idx);
    }

    // find the largest connected component
    inliers_indices_ = extractLargestConnectedComponentFromBitmap(bitmap);
  }

 private:
  inline bool distanceCheck(Eigen::Vector3f point, double threshold) {
    return std::abs(normal_.dot(point - point_)) < threshold;
  }
  inline bool normalCheck(Eigen::Vector3f normal, double threshold) {
    return acos(std::abs(normal_.dot(normal))) < threshold;
  }

  Eigen::Vector3f point_;
  Eigen::Vector3f normal_;
};

}  // namespace efficient_ransac
