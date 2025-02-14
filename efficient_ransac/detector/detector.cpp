#include "detector.h"

namespace efficient_ransac {

Detector::Detector() {}

int Detector::detect(const std::filesystem::path &filepath) {
  // params
  // TODO: see original code for an automatic way to determine the number of
  // subsets
  const size_t num_subsets = 10;
  // resolution of the octree
  const float octree_resolution = 0.01f;
  const float success_probability_threshold = 0.99f;

  // load the point cloud
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (pcl::io::loadPLYFile<pcl::PointXYZ>(filepath.string(), *cloud) == -1)
    return -1;

  // visualize the point cloud
  viewer_.showCloud(cloud);

  // empty set of candidate shapes
  std::vector<std::unique_ptr<Shape>> candidate_shapes;

  // create random subsets of the point cloud
  auto subsets = std::array<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>,
                            num_subsets>();
  for (size_t i = 0; i < num_subsets; i++) {
    // create a new subset
    subsets[i] = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    // reserve space
    subsets[i]->reserve(cloud->size() / num_subsets);
    // fill the subset
    for (size_t j = 0; j < cloud->size() / num_subsets; j++) {
      // add a random point to the subset
      subsets[i]->push_back(cloud->at(rand() % cloud->size()));
    }
  }

  // create an octree for each subset
  auto subset_octrees = std::array<
      std::unique_ptr<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>>,
      num_subsets>();
  for (size_t i = 0; i < num_subsets; i++) {
    // create a new octree
    subset_octrees[i] =
        std::make_unique<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>>(
            octree_resolution);
    // set the input cloud
    subset_octrees[i]->setInputCloud(subsets[i]);
    // build the octree
    subset_octrees[i]->addPointsFromInputCloud();
  }

  // create a global octree
  std::unique_ptr<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>>
      global_octree =
          std::make_unique<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>>(
              octree_resolution);
  // set the input cloud
  global_octree->setInputCloud(cloud);
  // build the octree
  global_octree->addPointsFromInputCloud();

  // main loop
  int candidate_size;
  int num_points;
  int drawn_candidates;
  int levels;
  int num_samples;
  float success_probability = std::pow(
      candidate_size / (num_points * levels * (1 << (num_samples - 1))),
      drawnCandidates);

  while (success_probability < success_probability_threshold) {
    // generate new candidate shapes

    // find the best candidate shape
    float best_score = 0.0f;
  }

  return 0;
}

}  // namespace efficient_ransac
