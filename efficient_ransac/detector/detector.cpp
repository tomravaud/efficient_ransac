#include "detector.h"


namespace efficient_ransac {

Detector::Detector() {srand(time(0));}

int Detector::detect(const std::filesystem::path &filepath) {
  // params
  // TODO: see original code for an automatic way to determine the number of
  // subsets
  // const size_t num_subsets = 10;
  // resolution of the octree
  // const float octree_resolution = 0.01f;
  const float success_probability_threshold = 0.99f;
  const int num_candidates = 100;
  const int num_point_candidates = 3;
  const thresholds thresholds = {0.01, 0.1};
  // load metadata of the point cloud
  auto metadata = std::make_shared<pcl::PCLPointCloud2>();
  if (pcl::io::loadPLYFile(filepath.string(), *metadata) == -1) return -1;

  // check if normals and curvature fields are present
  if (pcl::getFieldIndex(*metadata, "normal_x") == -1 ||
      pcl::getFieldIndex(*metadata, "normal_y") == -1 ||
      pcl::getFieldIndex(*metadata, "normal_z") == -1 ||
      pcl::getFieldIndex(*metadata, "curvature") == -1) {
    std::cout << "Point cloud does not have normals and/or curvature"
              << std::endl;
    return -1;
  }

  // load the point cloud
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
  if (pcl::io::loadPLYFile<pcl::PointNormal>(filepath.string(), *cloud) == -1)
    return -1;

  // visualize the point cloud
  viewer_.showCloud(cloud, true);

  // empty set of candidate shapes
  std::vector<std::shared_ptr<Shape>> candidate_shapes;
  std::vector<std::shared_ptr<Shape>> extracted_shapes;
  std::vector<bool> remaining_points(cloud->size(), true);
  // repeat:
  // * get N valid shapes
  // * find inliers for each shape
  // * find the best shape and keep it if good enough
  // * if good enough update the point cloud
  int num_valid_shapes = 0;
  while (num_valid_shapes < num_candidates) {
    // Sample points
    std::set<int> unique_indices;
    while (unique_indices.size() < 3) {
        int random_index = rand() % cloud->size();
        if (remaining_points[random_index]){
          unique_indices.insert(random_index); // Only inserts unique values
        }
    }
    std::vector<pcl::PointNormal> candidate_points;
    for (int index : unique_indices){
        candidate_points.push_back(cloud->at(index));
    }
    // generate a new candidate shape
    auto candidate_shape = std::make_unique<Plane>(candidate_points);
    // check if the candidate shape is valid
    if (candidate_shape->isValid(candidate_points,thresholds)) {
      num_valid_shapes++;
      candidate_shapes.push_back(std::move(candidate_shape));
    }
  }
  int best_shape_index = -1;
  int best_shape_num_inliers = 0;
  std::vector<int> best_shape_inliers;
  for (int i = 0; i < num_candidates; i++) {
    // find inliers for the candidate shape
    candidate_shapes[i]->computeInliersIndices(cloud,thresholds);
    auto inliers = candidate_shapes[i]->inliersIndices();
    // check if the candidate shape is the best
    if (inliers.size() > best_shape_num_inliers) {
      best_shape_index = i;
      best_shape_num_inliers = inliers.size();
      best_shape_inliers = inliers;
    }
  }
  int cloud_size = std::count(remaining_points.begin(), remaining_points.end(), true);
  if (1-pow(1-pow((float)best_shape_inliers.size()/cloud_size,num_point_candidates),candidate_shapes.size()) > success_probability_threshold){
    // update the point cloud
    for (auto index : best_shape_inliers) {
      remaining_points[index] = false;
    }  
    extracted_shapes.push_back(std::move(candidate_shapes[best_shape_index]));
    std::vector<std::shared_ptr<Shape>> candidate_shapes_tmp;
    for (auto candidate_shape: candidate_shapes){
      bool conflict = false;
      for (int inlier: best_shape_inliers){
        if (std::find(candidate_shape->inliersIndices().begin(), candidate_shape->inliersIndices().end(), inlier)!=candidate_shape->inliersIndices().end()){
          conflict = true;
          break;
        }
      if (!conflict) candidate_shapes_tmp.push_back(std::move(candidate_shape));
      }
    }
    candidate_shapes = candidate_shapes_tmp;
  }

  // create random subsets of the point cloud
  // auto subsets = std::array<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>,
  //                           num_subsets>();
  // for (size_t i = 0; i < num_subsets; i++) {
  //   // create a new subset
  //   subsets[i] = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  //   // reserve space
  //   subsets[i]->reserve(cloud->size() / num_subsets);
  //   // fill the subset
  //   for (size_t j = 0; j < cloud->size() / num_subsets; j++) {
  //     // add a random point to the subset
  //     subsets[i]->push_back(cloud->at(rand() % cloud->size()));
  //   }
  // }

  // create an octree for each subset
  // auto subset_octrees = std::array<
  //     std::unique_ptr<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>>,
  //     num_subsets>();
  // for (size_t i = 0; i < num_subsets; i++) {
  //   // create a new octree
  //   subset_octrees[i] =
  //       std::make_unique<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>>(
  //           octree_resolution);
  //   // set the input cloud
  //   subset_octrees[i]->setInputCloud(subsets[i]);
  //   // build the octree
  //   subset_octrees[i]->addPointsFromInputCloud();
  // }

  // // create a global octree
  // std::unique_ptr<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>>
  //     global_octree =
  //         std::make_unique<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>>(
  //             octree_resolution);
  // // set the input cloud
  // global_octree->setInputCloud(cloud);
  // // build the octree
  // global_octree->addPointsFromInputCloud();

  // main loop
  // int candidate_size;
  // int num_points;
  // int drawn_candidates;
  // int levels;
  // int num_samples;
  // float success_probability = std::pow(
  //     candidate_size / (num_points * levels * (1 << (num_samples - 1))),
  //     drawnCandidates);

  // while (success_probability < success_probability_threshold) {
  //   // generate new candidate shapes

  //   // find the best candidate shape
  //   float best_score = 0.0f;
  // }

  return 0;
}

}  // namespace efficient_ransac
