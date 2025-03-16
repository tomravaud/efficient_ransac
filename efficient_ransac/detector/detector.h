#pragma once

// C++ standard libraries
#include <cstdlib>  // for rand() and srand()
#include <ctime>
#include <filesystem>
#include <random>
#include <set>

// pcl
#include <pcl/io/ply_io.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PointIndices.h>

// yaml-cpp
#include <yaml-cpp/yaml.h>

// efficient_ransac
#include "../shapes/cylinder.h"
#include "../shapes/plane.h"
#include "../shapes/shape.h"
#include "../shapes/sphere.h"
#include "../utils/timer.h"

namespace efficient_ransac {

struct DetectorParams {
  float success_probability_threshold;
  int num_candidates;
  int num_point_candidates;
  int num_inliers_min;
  int max_num_shapes;
  bool use_localized_sampling;
  Thresholds thresholds;
  CellSize bitmap_cell_size;
  bool use_subsampling;

  static DetectorParams fromYAML(const std::filesystem::path &config_path) {
    YAML::Node config = YAML::LoadFile(config_path.string());
    DetectorParams params;
    params.success_probability_threshold =
        config["success_probability_threshold"].as<float>(0.99f);
    params.num_candidates = config["num_candidates"].as<int>(100);
    params.num_point_candidates = config["num_point_candidates"].as<int>(3);
    params.num_inliers_min = config["num_inliers_min"].as<int>(50);
    params.max_num_shapes = config["max_num_shapes"].as<int>(3);
    params.use_localized_sampling =
      config["use_localized_sampling"].as<bool>(true);
    params.thresholds.distance =
      config["thresholds"]["distance"].as<float>(0.1f);
    params.thresholds.normal = config["thresholds"]["normal"].as<float>(0.2f);
    params.bitmap_cell_size.x = config["bitmap_cell_size"].as<float>(0.1f);
    params.bitmap_cell_size.y = config["bitmap_cell_size"].as<float>(0.1f);
    params.use_subsampling =
        config["use_subsampling"].as<bool>(true);
    return params;
  }
};

class Detector {
 public:
  Detector(const std::filesystem::path &config_path);
  int detect(const std::filesystem::path &input_path,
             const std::filesystem::path &output_path);

 private:
  void randomSampling(
    std::set<int> &unique_indices,
    int num_point_candidates,
    const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud,
    const std::vector<bool> &remaining_points,
    std::mt19937 &gen,
    std::discrete_distribution<> &dist);
  bool localizedSampling(
    std::set<int> &unique_indices,
    int &random_depth,
    int num_point_candidates,
    const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud,
    const std::vector<bool> &remaining_points,
    const std::shared_ptr<pcl::octree::OctreePointCloudSearch<pcl::PointNormal>> &octree,
    std::mt19937 &gen,
    std::discrete_distribution<> &dist_first_point,
    std::discrete_distribution<> &dist_depth);

  inline bool randomAcceptance(
    int num_inliers, 
    int cloud_size, 
    int num_point_candidates, 
    int num_shape_candidates, 
    float success_probability_threshold){
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
        return 1 - pow(1 - (float)num_inliers / (cloud_size*max_depth*pow(2,num_point_candidates-1)),
                num_shape_candidates) >
        success_probability_threshold;
    }

  DetectorParams params_;
};

}  // namespace efficient_ransac
