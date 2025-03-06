#include "detector.h"

namespace efficient_ransac {

Detector::Detector() { srand(time(0)); }

int Detector::detect(const std::filesystem::path& input_path,
                     const std::filesystem::path& output_path) {
  // load metadata of the point cloud
  auto metadata = std::make_shared<pcl::PCLPointCloud2>();
  if (pcl::io::loadPLYFile(input_path.string(), *metadata) == -1) return -1;

  // check if normals and curvature fields are present
  if (pcl::getFieldIndex(*metadata, "normal_x") == -1 ||
      pcl::getFieldIndex(*metadata, "normal_y") == -1 ||
      pcl::getFieldIndex(*metadata, "normal_z") == -1 ||
      pcl::getFieldIndex(*metadata, "curvature") == -1) {
    std::cerr << "Point cloud does not have normals and/or curvature\n";
    return -1;
  }

  // load the point cloud
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
  if (pcl::io::loadPLYFile<pcl::PointNormal>(input_path.string(), *cloud) == -1)
    return -1;

  // build an octree of the cloud
  const float resolution = 0.1f;
  pcl::octree::OctreePointCloudSearch<pcl::PointNormal> octree(resolution);
  octree.setInputCloud(cloud);
  octree.addPointsFromInputCloud();
  int max_depth = octree.getTreeDepth();
  std::vector<int> depth_scores(max_depth, 0);

  //  params
  const float success_probability_threshold = 0.99f;
  const int num_candidates = 100;
  const int num_point_candidates = 3;
  const int num_inliers_min = 50;
  const thresholds thresholds = {
      /* distance = */ 0.2,  // [m]
      /* normal = */ 1.5     // [rad]
  };
  const int max_num_shapes = 3;

  // TODO: load the config file
  //  std::string config_path = "configs/config.yaml";
  //  YAML::Node config = YAML::LoadFile(config_path);

  std::vector<std::shared_ptr<Shape>> candidate_shapes;
  std::vector<std::shared_ptr<Shape>> extracted_shapes;

  // keep track of the remaining points
  std::vector<bool> remaining_points(cloud->size(), true);

  // main loop
  std::clog << "[INFO] Detecting shapes...\n";
  while (1 - pow(1 - pow((float)num_inliers_min /
                             std::count(remaining_points.begin(),
                                        remaining_points.end(), true),
                         num_point_candidates),
                 candidate_shapes.size()) <
             success_probability_threshold &&
         extracted_shapes.size() < max_num_shapes) {
    // generate a given number of valid candidate shapes
    int num_valid_shapes = 0;
    while (num_valid_shapes < num_candidates) {
      //--------------Random Sampling-----------------------------------------------------------
      //sample a minimal set of points to define a candidate shape
      // std::set<int> unique_indices;
      // while (unique_indices.size() < num_point_candidates) {
      //   int random_index = rand() % cloud->size();
      //   if (remaining_points[random_index])
      //     unique_indices.insert(random_index);  // Only inserts unique values
      // }
      //------------------------------------------------------------------------------------------

      //--------------Localized Sampling---------------------------------------------------------
      std::vector<int> remaining_indices;
      for (int i = 0; i < cloud->size(); i++) {
        if (remaining_points[i]) remaining_indices.push_back(i);
      }
      // Get first random index of a remaining point
      std::set<int> unique_indices;
      int first_point_index = remaining_indices[rand() % remaining_indices.size()];
      unique_indices.insert(first_point_index);

      // Get the key of the voxel containing the first point at a random depth
      // TODO: Change the prob of sampling depth
      int random_depth = rand() % max_depth;
      
      // Get the point indices of the voxel
      std::vector<int> point_indices_voxel;
      pcl::PointNormal query_point = cloud->at(first_point_index);
      // Calculate the voxel center containing the query point
      double voxel_size = octree.getVoxelSquaredSideLen(random_depth);
      double voxel_min_x = std::floor(query_point.x / voxel_size) * voxel_size;
      double voxel_min_y = std::floor(query_point.y / voxel_size) * voxel_size;
      double voxel_min_z = std::floor(query_point.z / voxel_size) * voxel_size;
      
      // Define the voxel boundaries
      Eigen::Vector3f voxel_min(voxel_min_x, voxel_min_y, voxel_min_z);
      Eigen::Vector3f voxel_max(voxel_min_x + voxel_size, voxel_min_y + voxel_size, voxel_min_z + voxel_size);
 
      
      // Find all points within this voxel using boxSearch
      int num_points_in_voxel = octree.boxSearch(voxel_min, voxel_max, point_indices_voxel);  
    
      if (num_points_in_voxel >= num_point_candidates) {
        // Remove points that have already been used
        point_indices_voxel.erase(
          std::remove_if(point_indices_voxel.begin(), point_indices_voxel.end(),
                        [&remaining_points](int idx) { return !remaining_points[idx]; }),
          point_indices_voxel.end());
        // Retrieve two random points from this voxel
        if (point_indices_voxel.size() >= 2) {
          while (unique_indices.size() < num_point_candidates) {
            unique_indices.insert(point_indices_voxel[rand() % point_indices_voxel.size()]);
          }
        }
        else {
          std::cout << "Not enough points in the voxel\n";
        }
      }
      //------------------------------------------------------------------------------------------

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
      candidate_shapes[i]->computeInliersIndices(cloud, thresholds,
                                                 remaining_points);
      auto inliers = candidate_shapes[i]->inliers_indices();
      // check if the candidate shape is the best
      if (inliers.size() > best_shape_inliers.size()) {
        best_shape_index = i;
        best_shape_inliers = inliers;
        // TODO: add inliers.size() to the depth of the voxel
      }
    }

    int cloud_size =
        std::count(remaining_points.begin(), remaining_points.end(), true);

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
                if (!candidate_shape) return true;  // protect against nullptr
                const auto& inliers = candidate_shape->inliers_indices();
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
  std::clog << "[INFO] Detected " << extracted_shapes.size() << " shapes\n";

  // label the point cloud
  auto cloud_labels = std::make_shared<pcl::PointCloud<pcl::PointXYZLNormal>>();
  pcl::copyPointCloud(*cloud, *cloud_labels);
  // default label = -1
  for (size_t i = 0; i < cloud->size(); i++) {
    cloud_labels->points[i].label = -1;
  }
  // label inliers
  for (size_t i = 0; i < extracted_shapes.size(); i++) {
    for (const auto& index : extracted_shapes[i]->inliers_indices()) {
      cloud_labels->points[index].label = i;
    }
  }

  // save the output
  pcl::PLYWriter writer;
  writer.write<pcl::PointXYZLNormal>(output_path.string(), *cloud_labels,
                                     /* binary_mode = */ true,
                                     /* use_camera = */ false);
  std::clog << "[INFO] Output saved to " << output_path << "\n";

  return 0;
}

}  // namespace efficient_ransac
