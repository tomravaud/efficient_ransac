#include "viewer.h"

namespace efficient_ransac {

Viewer::Viewer() { visualizer_ = nullptr; }

void Viewer::setup() {
  if (visualizer_) {
    return;
  }
  visualizer_ = std::make_shared<pcl::visualization::PCLVisualizer>("Viewer");
  visualizer_->setBackgroundColor(0, 0, 0);
  visualizer_->addCoordinateSystem(0.05);
  visualizer_->initCameraParameters();
  visualizer_->setCameraPosition(0, 0, 3,  // position
                                 0, 0, 0,  // viewpoint
                                 0, -1, 0  // up
  );
  visualizer_->addText("Press 'q' to quit", 10, 20, 20, 1, 1, 1, "quit");
  visualizer_->addText("Press 'r' to reset camera", 10, 40, 20, 1, 1, 1,
                       "reset");
}

void Viewer::showCloud(
    const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud,
    bool show_normals) {
  if (!visualizer_) setup();

  if (show_normals) {  // color the normals
    auto cloud_rgb = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::copyPointCloud(*cloud, *cloud_rgb);
    for (size_t i = 0; i < cloud->size(); i++) {
      cloud_rgb->at(i).r = (cloud->at(i).normal_x + 1) / 2 * 255;
      cloud_rgb->at(i).g = (cloud->at(i).normal_y + 1) / 2 * 255;
      cloud_rgb->at(i).b = (cloud->at(i).normal_z + 1) / 2 * 255;
    }
    visualizer_->addPointCloud(cloud_rgb, "cloud");
  } else {  // show the raw cloud
    auto cloud_xyz = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::copyPointCloud(*cloud, *cloud_xyz);
    visualizer_->addPointCloud(cloud_xyz, "cloud");
  }
  // set the point size
  visualizer_->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

  // run the visualizer
  while (!visualizer_->wasStopped()) {
    visualizer_->spinOnce(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void Viewer::showCloud(const std::filesystem::path &filepath,
                       bool show_normals) {
  if (!visualizer_) {
    setup();
  }

  // load metadata of the point cloud
  auto metadata = std::make_shared<pcl::PCLPointCloud2>();
  if (pcl::io::loadPLYFile(filepath.string(), *metadata) == -1) return;

  // check if normals and curvature fields are present
  if (pcl::getFieldIndex(*metadata, "normal_x") == -1 ||
      pcl::getFieldIndex(*metadata, "normal_y") == -1 ||
      pcl::getFieldIndex(*metadata, "normal_z") == -1 ||
      pcl::getFieldIndex(*metadata, "curvature") == -1) {
    std::cout << "Point cloud does not have normals and/or curvature"
              << std::endl;
    return;
  }

  // load the point cloud
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
  if (pcl::io::loadPLYFile<pcl::PointNormal>(filepath.string(), *cloud) == -1)
    return;

  showCloud(cloud, show_normals);
}

}  // namespace efficient_ransac
