#include "sphere.h"

namespace efficient_ransac {

Sphere::Sphere(std::vector<pcl::PointNormal> candidate_points,
               Thresholds thresholds, CellSize cell_size)
    : Shape(candidate_points, thresholds, cell_size) {
  // compute the sphere parameters
  Eigen::Vector3f p0 = candidate_points[0].getVector3fMap();
  Eigen::Vector3f p1 = candidate_points[1].getVector3fMap();
  Eigen::Vector3f n0 = candidate_points[0].getNormalVector3fMap().normalized();
  Eigen::Vector3f n1 = candidate_points[1].getNormalVector3fMap().normalized();

  // find the closest points on the two lines
  // (extremities of the perpendicular segment)
  Eigen::Vector3f d = p1 - p0;
  float a = n0.dot(n0);
  float b = n0.dot(n1);
  float c = n1.dot(n1);
  float d1 = n0.dot(d);
  float d2 = n1.dot(d);

  float denom = b * b - a * c;

  // lines are parallel or too close to define a unique sphere
  if (std::abs(denom) < 1e-6) {
    center_ = Eigen::Vector3f::Zero();
    radius_ = 0.0f;
    return;
  }

  float t0 = (b * d2 - c * d1) / denom;
  float t1 = (a * d2 - b * d1) / denom;

  Eigen::Vector3f q0 = p0 + t0 * n0;
  Eigen::Vector3f q1 = p1 + t1 * n1;

  // compute the center of the sphere (midpoint of the two closest points)
  center_ = (q0 + q1) / 2.0f;

  // compute the radius of the sphere
  float r0 = (p0 - center_).norm();
  float r1 = (p1 - center_).norm();
  radius_ = (r0 + r1) / 2.0f;

  // filter too small radius
  if (radius_ < 2 * cell_size_.y) {
    center_ = Eigen::Vector3f::Zero();
    radius_ = 0.0f;
    return;
  }
}

bool Sphere::isValid(std::vector<pcl::PointNormal> candidate_points) {
  for (auto point : candidate_points) {
    if (radius_ == 0.0f || angle(point) >= thresholds_.normal ||
        distance(point) >= thresholds_.distance)
      return false;
  }
  return true;
}

void Sphere::extractLargestConnectedComponent(
    const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud) {
  // compute 1 common bitmap for the upper and lower hemispheres
  // (in the original implementation, 2 bitmaps were used)
  std::unordered_map<CellCoord, std::vector<int>, CellCoordHasher> bitmap;

  float scaled_cell_size_x = cell_size_.x / (2 * radius_);
  float scaled_cell_size_y = cell_size_.y / (2 * radius_);

  const int grid_size_x =
      static_cast<int>(std::floor((1 / scaled_cell_size_x)));
  const int grid_size_y =
      static_cast<int>(std::floor((1 / scaled_cell_size_y)));

  scaled_cell_size_x = 1.0f / grid_size_x;
  scaled_cell_size_y = 1.0f / grid_size_y;

  for (int idx : inliers_indices_) {
    Eigen::Vector3f local_coord = cloud->at(idx).getVector3fMap() - center_;
    local_coord.normalize();  // the point should belong to the unit sphere
    // ensure the point is above the xy plane to have a valid projection
    float z = local_coord.z();  // store the z coordinate for hemisphere check
    local_coord.z() = std::abs(z);

    // project the point onto the unit disk and then onto the unit square
    std::pair<float, float> coord_disk, coord_square;
    hemisphere2Disk(local_coord, &coord_disk);
    disk2Square(coord_disk, &coord_square);

    // cell coordinates
    int cx =
        static_cast<int>(std::floor(coord_square.first / scaled_cell_size_x));
    int cy =
        static_cast<int>(std::floor(coord_square.second / scaled_cell_size_y));
    CellCoord key = {cx, cy};

    if (z >= 0) {  // upper hemisphere
      bitmap[key].push_back(idx);
    } else {                             // lower hemisphere
      key.x = 2 * grid_size_x - cx - 1;  // horizontal flip and shift
      bitmap[key].push_back(idx);
    }
  }

  // find the largest connected component
  inliers_indices_ = extractLargestConnectedComponentFromBitmapSphere(
      bitmap, 2 * grid_size_x, grid_size_y);
}

void Sphere::refit(
    const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud,
    const std::shared_ptr<pcl::octree::OctreePointCloudSearch<pcl::PointNormal>>
        &octree,
    const std::vector<bool> &remaining_points) {
  // compute inliers with an increased threshold
  thresholds_.distance *= 3;
  computeInliersIndices(cloud, octree, remaining_points, true);
  thresholds_.distance /= 3;

  // initial parameters
  Eigen::VectorXf params(4);
  params << center_.x(), center_.y(), center_.z(), radius_;

  // functor to optimize
  SphereFittingFunctor functor(cloud, inliers_indices_);

  // Levenberg-Marquardt optimization
  Eigen::LevenbergMarquardt<SphereFittingFunctor, float> lm(functor);
  int ret = lm.minimize(params);

  // update the parameters
  center_ = Eigen::Vector3f(params[0], params[1], params[2]);
  radius_ = params[3];

  // recompute the inliers
  computeInliersIndices(cloud, octree, remaining_points, true);
}

}  // namespace efficient_ransac
