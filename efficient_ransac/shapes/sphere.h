#pragma once

#include "shape.h"

namespace efficient_ransac {

struct SphereFittingFunctor {
  SphereFittingFunctor(
      const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud,
      const std::vector<int> &inliers_indices)
      : cloud_(cloud),
        inliers_indices_(inliers_indices),
        m(inliers_indices.size()),
        n(4) {}

  int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const {
    // retrieve the sphere parameters
    Eigen::Vector3f center(x[0], x[1], x[2]);
    float radius = x[3];

    // compute the residuals
    for (size_t i = 0; i < values(); i++) {
      Eigen::Vector3f point = cloud_->at(inliers_indices_[i]).getVector3fMap();
      fvec[i] = (point - center).norm() - radius;
    }
    return 0;
  }

  int df(const Eigen::VectorXf &x, Eigen::MatrixXf &fjac) const {
    // retrieve the sphere parameters
    Eigen::Vector3f center(x[0], x[1], x[2]);
    float radius = x[3];

    // compute the Jacobian [m x n]
    for (size_t i = 0; i < values(); i++) {
      Eigen::Vector3f point = cloud_->at(inliers_indices_[i]).getVector3fMap();
      Eigen::Vector3f diff = point - center;

      float dist = diff.norm();
      Eigen::Vector3f diff_normalized = diff / dist;

      fjac(i, 0) = -diff_normalized.x();  // d/dCx
      fjac(i, 1) = -diff_normalized.y();  // d/dCy
      fjac(i, 2) = -diff_normalized.z();  // d/dCz
      fjac(i, 3) = -1;                    // d/dR
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

class Sphere final : public Shape {
 public:
  Sphere(std::vector<pcl::PointNormal> candidate_points, Thresholds thresholds,
         CellSize cell_size);

  bool isValid(std::vector<pcl::PointNormal> candidate_points) override;

  void refit(const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud,
             const std::shared_ptr<
                 pcl::octree::OctreePointCloudSearch<pcl::PointNormal>> &octree,
             const std::vector<bool> &remaining_points) override;

 private:
  inline float distance(pcl::PointNormal point) override {
    return std::abs((point.getVector3fMap() - center_).norm() - radius_);
  }
  inline float angle(pcl::PointNormal point) override {
    Eigen::Vector3f normal_sphere =
        (point.getVector3fMap() - center_).normalized();
    Eigen::Vector3f normal_point = point.getNormalVector3fMap().normalized();
    return acos(std::abs(normal_sphere.dot(normal_point)));
  }
  void extractLargestConnectedComponent(
      const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud) override;

  /**
   * @brief Project a point from the unit hemisphere to the unit disk.CellCoord
   * Code from:
   * https://github.com/alessandro-gentilini/Efficient-RANSAC-for-Point-Cloud-Shape-Detection/blob/master/Sphere.cpp#L521
   *
   * @param p  point in the unit hemisphere
   * @param coord_disk  coordinates in the unit disk
   */
  inline void hemisphere2Disk(const Eigen::Vector3f &p,
                              std::pair<float, float> *coord_disk) const {
    coord_disk->first = std::sqrt(1 - p[2]);
    coord_disk->second = std::atan2(p[1], p[0]);
  }

  /**
   * @brief Project a point from the unit disk to the unit square
   * Method from:
   * A Low Distortion Map Between Disk and Square, P. Shirley et al., 1997
   * Code from:
   * https://github.com/alessandro-gentilini/Efficient-RANSAC-for-Point-Cloud-Shape-Detection/blob/master/Sphere.cpp#L528
   *
   * @param coord_disk  coordinates in the unit disk
   * @param coord_square  coordinates in the unit square
   */
  inline void disk2Square(const std::pair<float, float> &coord_disk,
                          std::pair<float, float> *coord_square) const {
    float r = coord_disk.first;
    float phi = coord_disk.second;
    float a, b;

    if (phi < float(-M_PI / 4.0)) phi += float(2 * M_PI);

    if (phi < float(M_PI / 4.0)) {
      a = r;
      b = phi * a / float(M_PI / 4.0);
    } else if (phi < float(3 * M_PI / 4.0)) {
      b = r;
      a = -(phi - float(M_PI / 2.0)) * b / float(M_PI / 4.0);
    } else if (phi < float(5 * M_PI / 4.0)) {
      a = -r;
      b = (phi - float(M_PI)) * a / float(M_PI / 4.0);
    } else {
      b = -r;
      a = -(phi - float(3 * M_PI / 2.0)) * b / float(M_PI / 4.0);
    }
    coord_square->first = (a + float(1.0)) / float(2.0);
    coord_square->second = (b + float(1.0)) / float(2.0);
  }

  // sphere parameters
  Eigen::Vector3f center_;
  float radius_;
};

}  // namespace efficient_ransac
