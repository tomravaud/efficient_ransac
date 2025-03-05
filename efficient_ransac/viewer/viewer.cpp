#include "viewer.h"

namespace efficient_ransac {

Viewer::Viewer() : is_initialized_(false), state_(State::Default) {}

void Viewer::setup() {
  if (is_initialized_) return;

  visualizer_ = std::make_shared<pcl::visualization::PCLVisualizer>("Viewer");
  visualizer_->setBackgroundColor(0, 0, 0);
  visualizer_->addCoordinateSystem(0.05);
  visualizer_->initCameraParameters();
  visualizer_->setCameraPosition(0, 0, 3,  // position
                                 0, 0, 0,  // viewpoint
                                 0, -1, 0  // up
  );
  std::clog << "[INFO] Viewer initialized\n";
  std::cout << "\n=== Controls ===\n"
            << "Press 'q' to quit\n"
            << "Press 'r' to reset camera\n";

  is_initialized_ = true;
}

void Viewer::showCloud(const std::filesystem::path &filepath) {
  setup();

  // load metadata of the point cloud
  auto metadata = std::make_shared<pcl::PCLPointCloud2>();
  if (pcl::io::loadPLYFile(filepath.string(), *metadata) == -1) return;

  bool has_normals = pcl::getFieldIndex(*metadata, "normal_x") != -1 &&
                     pcl::getFieldIndex(*metadata, "normal_y") != -1 &&
                     pcl::getFieldIndex(*metadata, "normal_z") != -1;
  bool has_labels = pcl::getFieldIndex(*metadata, "label") != -1;

  // load the point cloud based on the available fields
  if (has_normals && has_labels) {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZLNormal>>();
    if (pcl::io::loadPLYFile<pcl::PointXYZLNormal>(filepath.string(), *cloud) ==
        -1)
      return;
    showCloud(cloud);
  } else if (has_normals) {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
    if (pcl::io::loadPLYFile<pcl::PointNormal>(filepath.string(), *cloud) == -1)
      return;
    showCloud(cloud);
  } else if (has_labels) {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZL>>();
    if (pcl::io::loadPLYFile<pcl::PointXYZL>(filepath.string(), *cloud) == -1)
      return;
    showCloud(cloud);
  } else {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(filepath.string(), *cloud) == -1)
      return;
    showCloud(cloud);
  }
}

template <typename PointT>
void Viewer::showCloud(const std::shared_ptr<pcl::PointCloud<PointT>> &cloud) {
  setup();

  // create a colored cloud
  auto cloud_rgb = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  pcl::copyPointCloud(*cloud, *cloud_rgb);
  for (auto &point : cloud_rgb->points) point.r = point.g = point.b = 255;

  // add the cloud to the viewer
  visualizer_->addPointCloud(cloud_rgb, "cloud");
  visualizer_->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

  // handle specific callbacks based on point type
  if constexpr (std::is_same_v<PointT, pcl::PointNormal> ||
                std::is_same_v<PointT, pcl::PointXYZLNormal>) {
    std::cout << "Press 'n' to toggle normals\n";
    visualizer_->registerKeyboardCallback(
        [this, cloud,
         cloud_rgb](const pcl::visualization::KeyboardEvent &event) {
          if (event.getKeySym() == "n" && event.keyDown()) {
            if (state_ == State::Default || state_ == State::Labels) {
              state_ = State::Normals;
              for (size_t i = 0; i < cloud->size(); i++) {
                // scale the normals from [-1, 1] to [0, 255] for visualization
                cloud_rgb->at(i).r =
                    static_cast<uint8_t>((cloud->at(i).normal_x + 1) / 2 * 255);
                cloud_rgb->at(i).g =
                    static_cast<uint8_t>((cloud->at(i).normal_y + 1) / 2 * 255);
                cloud_rgb->at(i).b =
                    static_cast<uint8_t>((cloud->at(i).normal_z + 1) / 2 * 255);
              }
            } else {
              state_ = State::Default;
              for (auto &point : cloud_rgb->points)
                point.r = point.g = point.b = 255;
            }
            visualizer_->updatePointCloud(cloud_rgb, "cloud");
          }
        });
  }
  if constexpr (std::is_same_v<PointT, pcl::PointXYZL> ||
                std::is_same_v<PointT, pcl::PointXYZLNormal>) {
    std::cout << "Press 'l' to toggle labels\n";
    visualizer_->registerKeyboardCallback(
        [this, cloud,
         cloud_rgb](const pcl::visualization::KeyboardEvent &event) {
          if (event.getKeySym() == "l" && event.keyDown()) {
            if (state_ == State::Default || state_ == State::Normals) {
              state_ = State::Labels;
              std::unordered_map<uint32_t,
                                 std::tuple<uint8_t, uint8_t, uint8_t>>
                  label_colors;
              label_colors[-1] = {0, 0, 0};  // color for unlabelled points
              for (size_t i = 0; i < cloud->size(); ++i) {
                uint32_t label = cloud->at(i).label;
                if (label_colors.find(label) == label_colors.end()) {
                  // generate a random color for each label
                  label_colors[label] = {static_cast<uint8_t>(rand() % 256),
                                         static_cast<uint8_t>(rand() % 256),
                                         static_cast<uint8_t>(rand() % 256)};
                }
                auto [r, g, b] = label_colors[label];
                cloud_rgb->at(i).r = r;
                cloud_rgb->at(i).g = g;
                cloud_rgb->at(i).b = b;
              }
            } else {
              state_ = State::Default;
              for (auto &point : cloud_rgb->points)
                point.r = point.g = point.b = 255;
            }
            visualizer_->updatePointCloud(cloud_rgb, "cloud");
          }
        });
  }
  runViewer();
}

void Viewer::runViewer() {
  setup();

  while (!visualizer_->wasStopped()) {
    visualizer_->spinOnce(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

}  // namespace efficient_ransac
