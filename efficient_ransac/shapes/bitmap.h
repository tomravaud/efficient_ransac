#pragma once

#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace efficient_ransac {

struct CellCoord {
  int x, y;
  bool operator==(const CellCoord &c) const { return x == c.x && y == c.y; }
};

struct CellCoordHasher {
  std::size_t operator()(const CellCoord &c) const {
    return std::hash<int>()(c.x) ^ std::hash<int>()(c.y);
  }
};

std::vector<int> extractLargestConnectedComponentFromBitmap(
    const std::unordered_map<CellCoord, std::vector<int>, CellCoordHasher>
        &bitmap);

}  // namespace efficient_ransac
