#include "detector.h"

namespace efficient_ransac {

Detector::Detector(const std::filesystem::path &config_path)
    : params_(DetectorParams::fromYAML(config_path)) {
  srand(time(0));
}

void Detector::randomSampling(
  std::set<int> &unique_indices,
  int num_point_candidates,
  const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud,
  const std::vector<bool> &remaining_points,
  std::mt19937 &gen,
  std::discrete_distribution<> &dist){  
  while (unique_indices.size() < num_point_candidates) {
    unique_indices.insert(dist(gen));  // Only inserts unique values
    // int random_index = rand() % cloud->size();
    // if (remaining_points[random_index])
    //   unique_indices.insert(random_index);  // Only inserts unique values
  }
}

bool Detector::localizedSampling(
  std::set<int> &unique_indices,
  int &random_depth,
  int num_point_candidates,
  const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud,
  const std::vector<bool> &remaining_points,
  const pcl::octree::OctreePointCloudSearch<pcl::PointNormal> &octree,
  std::mt19937 &gen,
  std::discrete_distribution<> &dist_first_point,
  std::discrete_distribution<> &dist_depth) {

  // Get the first point at random
  int first_point_index = dist_first_point(gen);

  unique_indices.insert(first_point_index);

  // Get the key of the voxel containing the first point at a random depth
  random_depth = dist_depth(gen);

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
  Eigen::Vector3f voxel_max(voxel_min_x + voxel_size, voxel_min_y + voxel_size,
                            voxel_min_z + voxel_size);

  // Find all points within this voxel using boxSearch
  int num_points_in_voxel =
      octree.boxSearch(voxel_min, voxel_max, point_indices_voxel);

  if (num_points_in_voxel >= num_point_candidates) {
    // Remove points that have already been used
    point_indices_voxel.erase(
        std::remove_if(
            point_indices_voxel.begin(), point_indices_voxel.end(),
            [&remaining_points](int idx) { return !remaining_points[idx]; }),
        point_indices_voxel.end());
    // Retrieve two random points from this voxel
    if (point_indices_voxel.size() >= num_point_candidates) {
      while (unique_indices.size() < num_point_candidates) {
        unique_indices.insert(
            point_indices_voxel[rand() % point_indices_voxel.size()]);
      }
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

int Detector::detect(const std::filesystem::path &input_path,
                     const std::filesystem::path &output_path) {
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

  // Vectors to update depth probability sampling
  std::vector<int> depth_scores(max_depth, 0);
  std::vector<double> probabilities(max_depth, 1. / max_depth);
  float uniform_rate = 0.1;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::discrete_distribution<> dist_depth(probabilities.begin(), probabilities.end());

  std::vector<std::shared_ptr<Shape>> candidate_shapes;
  std::vector<std::shared_ptr<Shape>> extracted_shapes;

  // shapes to be used for detection
  std::vector<std::function<std::shared_ptr<Shape>(
      const std::vector<pcl::PointNormal> &)>>
      shape_constructors = {
          [this](const std::vector<pcl::PointNormal> &pts) {
            return std::make_shared<Plane>(pts, this->params_.thresholds,
                                           this->params_.bitmap_cell_size);
          },
          [this](const std::vector<pcl::PointNormal> &pts) {
            return std::make_shared<Sphere>(pts, this->params_.thresholds,
                                            this->params_.bitmap_cell_size);
          },
          [this](const std::vector<pcl::PointNormal> &pts) {
            return std::make_shared<Cylinder>(pts, this->params_.thresholds,
                                              this->params_.bitmap_cell_size);
          }};

  // keep track of the remaining points
  std::vector<bool> remaining_points(cloud->size(), true);

  // main loop
  std::clog << "[INFO] Detecting shapes...\n";
  Timer timer("Detection");

  while (1 - pow(1 - pow((float)params_.num_inliers_min /
                             std::count(remaining_points.begin(),
                                        remaining_points.end(), true),
                         params_.num_point_candidates),
                 candidate_shapes.size()) <
             params_.success_probability_threshold &&
         extracted_shapes.size() < params_.max_num_shapes &&
         cloud->size() > 0) {
    // generate a given number of valid candidate shapes
    int num_valid_shapes = 0;
    std::discrete_distribution<> dist(remaining_points.begin(), remaining_points.end());
    while (num_valid_shapes < params_.num_candidates) {
      // sample a set of points
      std::set<int> unique_indices;
      int random_depth;
      if (params_.use_localized_sampling){
        if(!localizedSampling(unique_indices, random_depth,
                              params_.num_point_candidates, cloud,
                              remaining_points, octree, gen, dist, dist_depth))
        continue;
      } else {
        randomSampling(unique_indices, params_.num_point_candidates, cloud,
                        remaining_points, gen, dist);
      }

      std::vector<pcl::PointNormal> candidate_points;
      for (int index : unique_indices) {
        candidate_points.push_back(cloud->at(index));
      }

      // auto candidate_shape = std::make_shared<Plane>(candidate_points);
      for (const auto &constructor : shape_constructors) {
        auto candidate_shape = constructor(candidate_points);

        // check if the candidate shape is valid
        if (candidate_shape->isValid(candidate_points)) {
          num_valid_shapes++;
          candidate_shape->computeInliersIndices(cloud, remaining_points);
          if (params_.use_localized_sampling)
            depth_scores[random_depth] +=
                candidate_shape->inliers_indices().size();
          candidate_shapes.push_back(std::move(candidate_shape));
        }
      }
    }

    if (params_.use_localized_sampling) {
      // Create a normalizing factor for the depth probabilities
      float weight = 0;
      for (int i = 0; i < max_depth; i++) {
        weight += depth_scores[i] / probabilities[i];
      }
      // Update the probabilities
      for (int i = 0; i < max_depth; i++) {
        probabilities[i] =
            (1-uniform_rate) * depth_scores[i] / probabilities[i] / weight +
             uniform_rate / max_depth;
      }
      dist_depth = std::discrete_distribution<>(probabilities.begin(), probabilities.end());
    }

    // find inliers for each candidate shape and keep the best one
    int best_shape_index = -1;
    int best_shape_num_inliers = 0;
    std::vector<int> best_shape_inliers;

    for (int i = 0; i < params_.num_candidates; i++) {
      auto inliers = candidate_shapes[i]->inliers_indices();
      // check if the candidate shape is the best
      if (inliers.size() > best_shape_inliers.size()) {
        best_shape_index = i;
        best_shape_inliers = inliers;
      }
    }

    int cloud_size =
        std::count(remaining_points.begin(), remaining_points.end(), true);

    // test if the best shape is good enough to be kept
    bool accept;
    if (params_.use_localized_sampling) {
      accept = localizedAcceptance(best_shape_inliers.size(), cloud->size(),
                                   params_.num_point_candidates,
                                   candidate_shapes.size(), max_depth,
                                   params_.success_probability_threshold);
    } else {
      accept = randomAcceptance(best_shape_inliers.size(), cloud->size(),
                                params_.num_point_candidates,
                                candidate_shapes.size(),
                                params_.success_probability_threshold);
    }
    if (accept) {
      // update the point cloud
      for (auto index : best_shape_inliers) remaining_points[index] = false;

      // add the best shape to the extracted shapes
      extracted_shapes.push_back(std::move(candidate_shapes[best_shape_index]));

      // TODO: Remove shared point instead of removing the shape candidate
      remove shapes sharing inliers with the best shape
      candidate_shapes.erase(
          std::remove_if(
              candidate_shapes.begin(), candidate_shapes.end(),
              [&best_shape_inliers](
                  const std::shared_ptr<Shape> &candidate_shape) {
                if (!candidate_shape) return true;  // protect against nullptr
                const auto &inliers = candidate_shape->inliers_indices();
                return std::any_of(
                    best_shape_inliers.begin(), best_shape_inliers.end(),
                    [&inliers](int inlier) {
                      return std::find(inliers.begin(), inliers.end(),
                                       inlier) != inliers.end();
                    });
              }),
          candidate_shapes.end());
      // for (auto &candidate_shape : candidate_shapes) {
      //   if (!candidate_shape) {
      //     std::cerr << "Null candidate_shape detected!" << std::endl;
      //     continue;
      //   }
      //   candidate_shape->removeFromInliersIndices(best_shape_inliers);
      // }
      // std::cout<<"success"<<std::endl;
      
      // candidate_shapes.erase(
      //     std::remove_if(candidate_shapes.begin(), candidate_shapes.end(),
      //         [this,&best_shape_inliers](const std::shared_ptr<Shape> &candidate_shape) {
      //           if (!candidate_shape) return true;  // protect against nullptr
      //           candidate_shape->removeFromInliersIndices(best_shape_inliers);
      //           return candidate_shape->inliers_indices().size() < this->params_.num_inliers_min;
      //         }),
      //     candidate_shapes.end());
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
    for (const auto &index : extracted_shapes[i]->inliers_indices()) {
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
