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
  void randomSampling(
    std::set<int> &unique_indices,
    int num_point_candidates,
    const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud,
    const std::vector<bool> &remaining_points);
  bool localizedSampling(
    std::set<int> &unique_indices,
    int &random_depth,
    int num_point_candidates,
    const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud,
    const std::vector<bool> &remaining_points,
    const std::vector<double> &probabilities,
    const pcl::octree::OctreePointCloudSearch<pcl::PointNormal> &octree,
    std::mt19937 &gen);
  inline bool randomAcceptance(
    int num_inliers, 
    int cloud_size, 
    int num_point_candidates, 
    int num_shape_candidates, 
    float success_probability_threshold){
        std::cout<<1 - pow(1 - pow((float)num_inliers / cloud_size,
        num_point_candidates),
num_shape_candidates)<<std::endl;
        return 1 - pow(1 - pow((float)num_inliers / cloud_size,
                        num_point_candidates),
                num_shape_candidates) >
        success_probability_threshold;
    }
  inline bool localizedAcceptance(
    int num_inliers, 
    int cloud_size, 
    int num_point_candidates, 
    int num_shape_candidates, 
    int max_depth,
    float success_probability_threshold){
        std::cout<<1 - pow(1 - (float)num_inliers / (cloud_size*max_depth*pow(2,num_point_candidates-1)),
        num_shape_candidates)<<std::endl;
        return 1 - pow(1 - (float)num_inliers / (cloud_size*max_depth*pow(2,num_point_candidates-1)),
                num_shape_candidates) >
        success_probability_threshold;
    }
};


}  // namespace efficient_ransac
