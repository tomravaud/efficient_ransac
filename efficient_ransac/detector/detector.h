#pragma once

#include <cstdlib>  // for rand() and srand()
#include <ctime>
#include <filesystem>
#include <set>

// pcl
#include <pcl/io/ply_io.h>
#include <pcl/octree/octree_search.h>

#include "../shapes/plane.h"
#include "../shapes/shape.h"

namespace efficient_ransac {

class Detector {
 public:
  Detector();
  int detect(const std::filesystem::path &input_path,
             const std::filesystem::path &output_path);
};

}  // namespace efficient_ransac
