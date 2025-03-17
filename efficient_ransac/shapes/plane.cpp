#include "plane.h"

namespace efficient_ransac {

Plane::Plane(std::vector<pcl::PointNormal> candidate_points,
             Thresholds thresholds, CellSize cell_size)
    : Shape(candidate_points, thresholds, cell_size) {
  // compute the plane parameters
  point_ = candidate_points[0].getVector3fMap();
  Eigen::Vector3f p0p1 = candidate_points[1].getVector3fMap() - point_;
  Eigen::Vector3f p0p2 = candidate_points[2].getVector3fMap() - point_;
  normal_ = p0p1.cross(p0p2);
  normal_.normalize();
}

bool Plane::isValid(std::vector<pcl::PointNormal> candidate_points) {
  for (auto point : candidate_points) {
    if (angle(point) >= thresholds_.normal) return false;
  }
  return true;
}

void Plane::extractLargestConnectedComponent(
    const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud) {
  // local coordinate system in the plane
  Eigen::Vector3f u, v;
  u = normal_.unitOrthogonal();
  v = normal_.cross(u);

  // fill the bitmap (only active cells are stored)
  std::unordered_map<CellCoord, std::vector<int>, CellCoordHasher> bitmap;
  for (int idx : inliers_indices_) {
    Eigen::Vector3f local_coords = cloud->at(idx).getVector3fMap() - point_;
    int cx = static_cast<int>(std::floor(local_coords.dot(u) / cell_size_.x));
    int cy = static_cast<int>(std::floor(local_coords.dot(v) / cell_size_.y));
    CellCoord key = {cx, cy};
    bitmap[key].push_back(idx);
  }

  // find the largest connected component
  inliers_indices_ = extractLargestConnectedComponentFromBitmap(bitmap);
}

void Plane::refit(
    const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud,
    const std::shared_ptr<pcl::octree::OctreePointCloudSearch<pcl::PointNormal>>
        &octree,
    const std::vector<bool> &remaining_points) {
  // compute inliers with an increased threshold
  thresholds_.distance *= 3;
  computeInliersIndices(cloud, octree, remaining_points, true);
  thresholds_.distance /= 3;

  // matrix of inliers
  const int num_inliers = inliers_indices_.size();
  Eigen::MatrixXf inliers(num_inliers, 3);

  // compute the barycenter of the inliers
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  for (int i = 0; i < num_inliers; i++) {
    Eigen::Vector3f point = cloud->at(inliers_indices_[i]).getVector3fMap();
    inliers.row(i) = point;
    centroid += point;
  }
  centroid /= num_inliers;

  // center the inliers
  inliers.rowwise() -= centroid.transpose();

  // compute the covariance matrix
  Eigen::Matrix3f covariance = (inliers.transpose() * inliers) / num_inliers;

  // eigen decomposition (with orthogonal matrix)
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);

  // update the plane parameters
  point_ = centroid;
  normal_ = solver.eigenvectors().col(0);  // smallest eigenvalue
  normal_.normalize();

  // recompute the inliers
  computeInliersIndices(cloud, octree, remaining_points, true);
}

}  // namespace efficient_ransac
