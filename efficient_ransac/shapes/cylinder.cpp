#include "cylinder.h"

namespace efficient_ransac {

Cylinder::Cylinder(std::vector<pcl::PointNormal> candidate_points,
                   Thresholds thresholds, CellSize cell_size)
    : Shape(candidate_points, thresholds, cell_size) {
  // compute the cylinder parameters
  Eigen::Vector3f p0 = candidate_points[0].getVector3fMap();
  Eigen::Vector3f p1 = candidate_points[1].getVector3fMap();
  Eigen::Vector3f n0 = candidate_points[0].getNormalVector3fMap().normalized();
  Eigen::Vector3f n1 = candidate_points[1].getNormalVector3fMap().normalized();

  // avoid normals too close
  if (n0.dot(n1) > cos(M_PI / 180.0f)) {  // 1 degree threshold
    axis_ = Eigen::Vector3f::Zero();
    center_ = Eigen::Vector3f::Zero();
    radius_ = 0.0f;
    return;
  }

  // direction of the axis
  axis_ = n0.cross(n1);
  axis_.normalize();

  // project the 2 points onto the plane orthogonal to the axis
  Eigen::Vector3f proj_p0 = p0 - p0.dot(axis_) * axis_;
  Eigen::Vector3f proj_p1 = p1 - p1.dot(axis_) * axis_;

  // find the intersection of the 2 lines
  Eigen::Vector3f d = proj_p1 - proj_p0;
  float a = n0.dot(n0);
  float b = n0.dot(n1);
  float c = n1.dot(n1);
  float d1 = n0.dot(d);
  float d2 = n1.dot(d);

  float denom = b * b - a * c;

  // avoid parallel lines
  if (std::abs(denom) < 1e-6) {
    axis_ = Eigen::Vector3f::Zero();
    center_ = Eigen::Vector3f::Zero();
    radius_ = 0.0f;
    return;
  }
  // center = proj_p0 + t0 * n0 = proj_p1 + t1 * n1
  float t0 = (b * d2 - c * d1) / denom;
  center_ = proj_p0 + t0 * n0;
  radius_ = std::abs(t0);

  // filter too small radius
  if (radius_ < 2 * cell_size_.y / (2 * M_PI)) {
    axis_ = Eigen::Vector3f::Zero();
    center_ = Eigen::Vector3f::Zero();
    radius_ = 0.0f;
    return;
  }
}

bool Cylinder::isValid(std::vector<pcl::PointNormal> candidate_points) {
  for (auto point : candidate_points) {
    if (radius_ == 0.0f || angle(point) >= thresholds_.normal ||
        distance(point) >= thresholds_.distance)
      return false;
  }
  return true;
}

void Cylinder::extractLargestConnectedComponent(
    const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud) {
  // local coordinate system in the cylinder
  Eigen::Vector3f u, v;
  u = axis_.unitOrthogonal();
  v = axis_.cross(u);

  WrapParams wrap_params;
  wrap_params.wrap_y = true;
  wrap_params.bitmap_size_y =
      static_cast<int>(std::floor((2 * M_PI * radius_ / cell_size_.y)));
  // update the cell size temporarily to close the cylinder
  float cell_size_y = 2 * M_PI * radius_ / wrap_params.bitmap_size_y;

  // fill the bitmap (only active cells are stored)
  std::unordered_map<CellCoord, std::vector<int>, CellCoordHasher> bitmap;
  for (int idx : inliers_indices_) {
    Eigen::Vector3f local_coords = cloud->at(idx).getVector3fMap() - center_;

    // compute cylindrical coordinates (radial is constant)
    float axial_coord = local_coords.dot(axis_);
    Eigen::Vector3f proj_local_coords = local_coords - axial_coord * axis_;
    float angular_coord =
        std::atan2(proj_local_coords.dot(v), proj_local_coords.dot(u));
    if (angular_coord < 0)
      angular_coord += 2 * M_PI;                 // from [-pi, pi] to [0, 2pi]
    float arc_length = radius_ * angular_coord;  // back to distance

    // cell coordinates
    int cx = static_cast<int>(std::floor(axial_coord / cell_size_.x));
    int cy = static_cast<int>(std::floor(arc_length / cell_size_y));
    CellCoord key = {cx, cy};
    bitmap[key].push_back(idx);
  }

  // find the largest connected component
  inliers_indices_ =
      extractLargestConnectedComponentFromBitmap(bitmap, wrap_params);
}

void Cylinder::refit(
    const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud,
    const std::shared_ptr<pcl::octree::OctreePointCloudSearch<pcl::PointNormal>>
        &octree,
    const std::vector<bool> &remaining_points) {
  // compute inliers with an increased threshold
  thresholds_.distance *= 3;
  computeInliersIndices(cloud, octree, remaining_points, true);
  thresholds_.distance /= 3;

  // initial parameters
  Eigen::VectorXf params(7);
  params << center_.x(), center_.y(), center_.z(), axis_.x(), axis_.y(),
      axis_.z(), radius_;

  // functor to optimize
  CylinderFittingFunctor functor(cloud, inliers_indices_);

  // Levenberg-Marquardt optimization
  Eigen::LevenbergMarquardt<CylinderFittingFunctor, float> lm(functor);
  int ret = lm.minimize(params);

  // update the parameters
  center_ = Eigen::Vector3f(params[0], params[1], params[2]);
  axis_ = Eigen::Vector3f(params[3], params[4], params[5]);
  axis_.normalize();
  radius_ = params[6];

  // recompute the inliers
  computeInliersIndices(cloud, octree, remaining_points, true);
}

}  // namespace efficient_ransac
