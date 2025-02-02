#pragma once

#include <chrono>
#include <filesystem>
#include <iostream>
#include <thread>

// pcl
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl;

class Viewer {
 public:
  Viewer();
  void setup();
  void showCloud(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud);
  void showCloud(const std::filesystem::path &filename);

 private:
  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
};
