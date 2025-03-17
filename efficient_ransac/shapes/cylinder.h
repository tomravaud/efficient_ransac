#pragma once

#include "shape.h"

namespace efficient_ransac {

struct CylinderFittingFunctor {
  CylinderFittingFunctor(
      const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud,
      const std::vector<int> &inliers_indices)
      : cloud_(cloud),
        inliers_indices_(inliers_indices),
        m(inliers_indices.size()),
        n(7) {}

  int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const {
    // retrieve the cylinder parameters
    Eigen::Vector3f center(x[0], x[1], x[2]);
    Eigen::Vector3f axis(x[3], x[4], x[5]);
    axis.normalize();
    float radius = x[6];

    // compute the residuals
    for (size_t i = 0; i < values(); i++) {
      Eigen::Vector3f point = cloud_->at(inliers_indices_[i]).getVector3fMap();
      Eigen::Vector3f v = point - center;
      Eigen::Vector3f proj = center + (v.dot(axis)) * axis;
      float dist_ortho = (point - proj).norm();
      fvec[i] = dist_ortho - radius;
    }

    return 0;
  }

  int df(const Eigen::VectorXf &x, Eigen::MatrixXf &fjac) const {
    // retrieve the cylinder parameters
    Eigen::Vector3f center(x[0], x[1], x[2]);
    Eigen::Vector3f axis(x[3], x[4], x[5]);
    axis.normalize();
    float radius = x[6];

    // compute the Jacobian [m x n]
    for (size_t i = 0; i < values(); i++) {
      Eigen::Vector3f point = cloud_->at(inliers_indices_[i]).getVector3fMap();
      Eigen::Vector3f v = point - center;
      float proj_scalar = v.dot(axis);
      Eigen::Vector3f proj = center + proj_scalar * axis;
      Eigen::Vector3f diff = point - proj;
      float dist = diff.norm();
      Eigen::Vector3f diff_normalized = diff / dist;

      // d/dCx, d/dCy, d/dCz
      fjac(i, 0) = -diff_normalized.x();
      fjac(i, 1) = -diff_normalized.y();
      fjac(i, 2) = -diff_normalized.z();

      // d/dAx, d/dAy, d/dAz
      Eigen::Vector3f d_deriv = -2 * proj_scalar * v / dist;
      fjac(i, 3) = d_deriv.x();
      fjac(i, 4) = d_deriv.y();
      fjac(i, 5) = d_deriv.z();

      // d/dR
      fjac(i, 6) = -1;
    }
    return 0;
  }

  // number of data points
  int m;
  int values() const { return m; }

  // number of parameters
  int n;
  int inputs() const { return n; }

  const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud_;
  const std::vector<int> &inliers_indices_;
};

class Cylinder final : public Shape {
 public:
  Cylinder(std::vector<pcl::PointNormal> candidate_points,
           Thresholds thresholds, CellSize cell_size);

  bool isValid(std::vector<pcl::PointNormal> candidate_points) override;

  void refit(const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud,
             const std::shared_ptr<
                 pcl::octree::OctreePointCloudSearch<pcl::PointNormal>> &octree,
             const std::vector<bool> &remaining_points) override;

 private:
  inline float distance(pcl::PointNormal point) override {
    // project the point onto the plane orthogonal to the axis
    Eigen::Vector3f p = point.getVector3fMap();
    Eigen::Vector3f proj_p = p - p.dot(axis_) * axis_;
    return std::abs((proj_p - center_).norm() - radius_);
  }
  inline float angle(pcl::PointNormal point) override {
    // project the point onto the plane orthogonal to the axis
    Eigen::Vector3f p = point.getVector3fMap();
    Eigen::Vector3f proj_p = p - p.dot(axis_) * axis_;
    // normal of the cylinder at the projected point
    Eigen::Vector3f normal_cylinder = (proj_p - center_).normalized();
    // normal of the point
    Eigen::Vector3f normal_point = point.getNormalVector3fMap().normalized();
    return std::acos(std::abs(normal_cylinder.dot(normal_point)));
  }
  void extractLargestConnectedComponent(
      const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud) override;

  // cylinder parameters
  Eigen::Vector3f axis_;
  Eigen::Vector3f center_;
  float radius_;
};

}  // namespace efficient_ransac
