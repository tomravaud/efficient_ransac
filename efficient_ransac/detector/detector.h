#pragma once

#include <filesystem>
#include <cstdlib>  // For rand() and srand()
#include <ctime>   
#include <set> 

// pcl
#include <pcl/io/ply_io.h>
#include <pcl/octree/octree_search.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "../shapes/shape.h"
#include "../shapes/plane.h"
#include "../viewer/viewer.h"

namespace efficient_ransac {

class Detector {
 public:
  Detector();
  int detect(const std::filesystem::path &filepath);

 private:
  Viewer viewer_;
};

}  // namespace efficient_ransac
