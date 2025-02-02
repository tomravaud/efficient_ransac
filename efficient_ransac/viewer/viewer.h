#pragma once

#include <iostream>
#include <chrono>
#include <thread>
#include <filesystem>

// pcl
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>


class Viewer {
public:
    Viewer();
    void setup();
    void showCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
    void showCloud(const std::filesystem::path &filename);

private:
    pcl::visualization::PCLVisualizer::Ptr viewer;
};
