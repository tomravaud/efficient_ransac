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
    const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud) {
  if (!visualizer_) {
    setup();
  }
  visualizer_->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
  visualizer_->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

  while (!visualizer_->wasStopped()) {
    visualizer_->spinOnce(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void Viewer::showCloud(const std::filesystem::path &filepath) {
  if (!visualizer_) {
    setup();
  }
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (pcl::io::loadPLYFile<pcl::PointXYZ>(filepath.string(), *cloud) == -1)
    return;

  showCloud(cloud);
}

}  // namespace efficient_ransac
