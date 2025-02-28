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
  void showCloud(const std::filesystem::path &filename);

 private:
  enum class State { Default, Normals, Labels };

  void setup();
  void runViewer();
  template <typename PointT>
  void showCloud(const std::shared_ptr<pcl::PointCloud<PointT>> &cloud);

  bool is_initialized_;
  std::shared_ptr<pcl::visualization::PCLVisualizer> visualizer_;
  State state_;
};

}  // namespace efficient_ransac
