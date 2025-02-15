#include "plane.h"

namespace efficient_ransac {

Plane::Plane(std::vector<pcl::PointXYZ> candidate_points,
             std::vector<pcl::Normal> candidate_normals)
    : Shape(candidate_points, candidate_normals) {
  // compute the plane parameters
  point_ = candidate_points[0];
  pcl::PointXYZ p0p1 = candidate_points[1] - candidate_points[0];
  pcl::PointXYZ p0p2 = candidate_points[2] - candidate_points[0];
  normal_ = cross(p0p1, p0p2);
  normalize(normal_);
}

bool Plane::isValid(std::vector<pcl::PointXYZ> candidate_points,
                    std::vector<pcl::Normal> candidate_normals,
                    thresholds thresholds) {
  for (size_t i = 0; i < candidate_points.size(); i++) {
    if (acos(abs(dot(normal_, candidate_normals[i]))) < thresholds.normal)
      return false;
  }
  return true;
}

std::vector<int> Plane::inliersIndices(
    const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
    const std::shared_ptr<pcl::PointCloud<pcl::Normal>> &normals,
    const thresholds thresholds) {
  std::vector<int> inliers_indices;
  for (size_t i = 0; i < cloud->size(); i++) {
    if (distanceCheck(cloud->at(i), thresholds.distance) &&
        normalCheck(normals->at(i), thresholds.normal))
      inliers_indices.push_back(i);
  }
  return inliers_indices;
}

}  // namespace efficient_ransac
