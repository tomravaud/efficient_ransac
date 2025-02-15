#pragma once

#include <chrono>
#include <filesystem>
#include <iostream>
#include <thread>

// pcl
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace efficient_ransac {

class Viewer {
 public:
  Viewer();
  void setup();
  void showCloud(
      const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud,
      bool show_normals = false);
  void showCloud(const std::filesystem::path &filename,
                 bool show_normals = false);

 private:
  std::shared_ptr<pcl::visualization::PCLVisualizer> visualizer_;
};

}  // namespace efficient_ransac
