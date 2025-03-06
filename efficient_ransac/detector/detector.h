#pragma once

// C++ standard libraries
#include <cstdlib>  // for rand() and srand()
#include <ctime>
#include <filesystem>
#include <set>
#include <random>

// pcl
#include <pcl/io/ply_io.h>
#include <pcl/octree/octree_search.h>

// yaml-cpp
// #include <yaml-cpp/yaml.h>

// efficient_ransac
#include "../shapes/plane.h"
#include "../shapes/shape.h"

namespace efficient_ransac {

class Detector {
 public:
  Detector();
  int detect(const std::filesystem::path &input_path,
             const std::filesystem::path &output_path);
 private:
  void random_sampling(
    std::set<int> &unique_indices,
    int num_point_candidates,
    const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud,
    const std::vector<bool> &remaining_points);
  bool localized_sampling(
    std::set<int> &unique_indices,
    int &random_depth,
    int num_point_candidates,
    const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud,
    const std::vector<bool> &remaining_points,
    const std::vector<double> &probabilities,
    const pcl::octree::OctreePointCloudSearch<pcl::PointNormal> &octree,
    std::mt19937 &gen);
};


}  // namespace efficient_ransac
