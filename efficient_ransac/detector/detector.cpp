#include "detector.h"

namespace efficient_ransac {

Detector::Detector() { srand(time(0)); }

int Detector::detect(const std::filesystem::path& filepath) {
  // load metadata of the point cloud
  auto metadata = std::make_shared<pcl::PCLPointCloud2>();
  if (pcl::io::loadPLYFile(filepath.string(), *metadata) == -1) return -1;

  // check if normals and curvature fields are present
  if (pcl::getFieldIndex(*metadata, "normal_x") == -1 ||
      pcl::getFieldIndex(*metadata, "normal_y") == -1 ||
      pcl::getFieldIndex(*metadata, "normal_z") == -1 ||
      pcl::getFieldIndex(*metadata, "curvature") == -1) {
    std::cerr << "Point cloud does not have normals and/or curvature"
              << std::endl;
    return -1;
  }

  // load the point cloud
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
  if (pcl::io::loadPLYFile<pcl::PointNormal>(filepath.string(), *cloud) == -1)
    return -1;

  // visualize the point cloud
  viewer_.showCloud(cloud, true);

  // TODO: move to config file
  //  params
  const float success_probability_threshold = 0.9f;
  // const float success_probability_threshold = 0.99f;
  const int num_candidates = 100;
  const int num_point_candidates = 3;
  const int num_inliers_min = 50;
  const thresholds thresholds = {0.01, 0.1};

  std::vector<std::shared_ptr<Shape>> candidate_shapes;
  std::vector<std::shared_ptr<Shape>> extracted_shapes;

  // keep track of the remaining points
  // NOTE: replace by a label field?
  std::vector<bool> remaining_points(cloud->size(), true);

  // main loop
  // while (1 - pow(1 - pow((float)num_inliers_min /
  //                            std::count(remaining_points.begin(),
  //                                       remaining_points.end(), true),
  //                        num_point_candidates),
  //                candidate_shapes.size()) <
  //        success_probability_threshold) {

  int i = 0;

  while (i < 1000) {
    std::cout << i << std::endl;
    i++;

    // generate a given number of valid candidate shapes
    int num_valid_shapes = 0;
    while (num_valid_shapes < num_candidates) {
      // sample a minimal set of points to define a candidate shape
      std::set<int> unique_indices;
      while (unique_indices.size() < num_point_candidates) {
        int random_index = rand() % cloud->size();
        if (remaining_points[random_index])
          unique_indices.insert(random_index);  // Only inserts unique values
      }
      std::vector<pcl::PointNormal> candidate_points;
      for (int index : unique_indices) {
        candidate_points.push_back(cloud->at(index));
      }
      // TODO: generate a new candidate for each shape type with the same
      // points? generate a new candidate shape
      auto candidate_shape = std::make_shared<Plane>(candidate_points);
      // check if the candidate shape is valid
      if (candidate_shape->isValid(candidate_points, thresholds)) {
        num_valid_shapes++;
        candidate_shapes.push_back(std::move(candidate_shape));
      }
    }

    // find inliers for each candidate shape and keep the best one
    int best_shape_index = -1;
    int best_shape_num_inliers = 0;
    std::vector<int> best_shape_inliers;

    for (int i = 0; i < num_candidates; i++) {
      candidate_shapes[i]->computeInliersIndices(cloud, thresholds);
      auto inliers = candidate_shapes[i]->inliersIndices();
      // check if the candidate shape is the best
      if (inliers.size() > best_shape_inliers.size()) {
        best_shape_index = i;
        best_shape_inliers = inliers;
      }
    }

    int cloud_size =
        std::count(remaining_points.begin(), remaining_points.end(), true);

    std::cout << 1 - pow(1 - pow((float)best_shape_inliers.size() / cloud_size,
                                 num_point_candidates),
                         candidate_shapes.size())
              << std::endl;

    // test if the best shape is good enough to be kept
    if (1 - pow(1 - pow((float)best_shape_inliers.size() / cloud_size,
                        num_point_candidates),
                candidate_shapes.size()) >
        success_probability_threshold) {
      // update the point cloud
      for (auto index : best_shape_inliers) remaining_points[index] = false;

      // add the best shape to the extracted shapes
      extracted_shapes.push_back(std::move(candidate_shapes[best_shape_index]));

      // remove shapes sharing inliers with the best shape
      candidate_shapes.erase(
          std::remove_if(
              candidate_shapes.begin(), candidate_shapes.end(),
              [&best_shape_inliers](
                  const std::shared_ptr<Shape>& candidate_shape) {
                if (!candidate_shape) return true;  // Protection contre nullptr
                const auto& inliers = candidate_shape->inliersIndices();
                return std::any_of(
                    best_shape_inliers.begin(), best_shape_inliers.end(),
                    [&inliers](int inlier) {
                      return std::find(inliers.begin(), inliers.end(),
                                       inlier) != inliers.end();
                    });
              }),
          candidate_shapes.end());
    }
  }
  return 0;
}

}  // namespace efficient_ransac
