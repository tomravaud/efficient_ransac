#include "plane.h"

namespace efficient_ransac {

Plane::Plane(std::vector<pcl::PointNormal> candidate_points)
    : Shape(candidate_points) {
  // compute the plane parameters
  point_ = candidate_points[0].getVector3fMap();
  Eigen::Vector3f p0p1 = candidate_points[1].getVector3fMap() - point_;
  Eigen::Vector3f p0p2 = candidate_points[2].getVector3fMap() - point_;
  normal_ = p0p1.cross(p0p2);
  normal_.normalize();
}

bool Plane::isValid(std::vector<pcl::PointNormal> candidate_points,
                    thresholds thresholds) {
  for (size_t i = 0; i < candidate_points.size(); i++) {
    if (!normalCheck(candidate_points[i].getNormalVector3fMap(),
                     thresholds.normal))
      return false;
  }
  return true;
}

void Plane::computeInliersIndices(
    const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud,
    const thresholds thresholds) {
  if (!(inliers_indices_.empty())) return;
  for (size_t i = 0; i < cloud->size(); i++) {
    if (distanceCheck(cloud->at(i).getVector3fMap(), thresholds.distance) &&
        normalCheck(cloud->at(i).getNormalVector3fMap(), thresholds.normal))
      inliers_indices_.push_back(i);
  }
}

}  // namespace efficient_ransac
