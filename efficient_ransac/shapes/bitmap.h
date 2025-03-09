#pragma once

#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace efficient_ransac {

struct CellSize {
  float x, y;
};

struct CellCoord {
  int x, y;
  bool operator==(const CellCoord &c) const { return x == c.x && y == c.y; }
};

struct CellCoordHasher {
  std::size_t operator()(const CellCoord &c) const {
    return std::hash<int>()(c.x) ^ std::hash<int>()(c.y);
  }
};

struct WrapParams {
  bool wrap_x = false;
  bool wrap_y = false;
  int bitmap_size_x = 0;
  int bitmap_size_y = 0;
};

std::vector<int> extractLargestConnectedComponentFromBitmap(
    const std::unordered_map<CellCoord, std::vector<int>, CellCoordHasher>
        &bitmap,
    const WrapParams &wrap_params = {false, false, 0, 0});

std::vector<int> extractLargestConnectedComponentFromBitmapSphere(
    const std::unordered_map<CellCoord, std::vector<int>, CellCoordHasher>
        &bitmap,
    const int bitmap_size_x, const int bitmap_size_y);

}  // namespace efficient_ransac
