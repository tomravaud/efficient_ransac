#include "bitmap.h"

namespace efficient_ransac {

std::vector<int> extractLargestConnectedComponentFromBitmap(
    const std::unordered_map<CellCoord, std::vector<int>, CellCoordHasher>
        &bitmap,
    const WrapParams &wrap_params) {
  // BFS to find the largest connected component
  std::vector<int> largest_component;
  std::unordered_set<CellCoord, CellCoordHasher> visited;

  for (const auto &[key, indices] : bitmap) {
    if (visited.count(key)) continue;  // already visited cell

    std::vector<int> current_component;
    std::queue<CellCoord> q;
    q.push(key);
    visited.insert(key);

    while (!q.empty()) {
      CellCoord current = q.front();
      q.pop();

      // add the current cell to the current component
      auto it = bitmap.find(current);
      if (it != bitmap.end()) {
        current_component.insert(current_component.end(), it->second.begin(),
                                 it->second.end());
      }

      // 8-neighbors exploration
      for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
          if (dx == 0 && dy == 0) continue;  // skip the current cell

          int neighbor_x = current.x + dx;
          int neighbor_y = current.y + dy;
          // handle wrapping, ie periodic boundary conditions
          if (wrap_params.wrap_x) {
            neighbor_x = (neighbor_x + wrap_params.bitmap_size_x) %
                         wrap_params.bitmap_size_x;
          }
          if (wrap_params.wrap_y) {
            neighbor_y = (neighbor_y + wrap_params.bitmap_size_y) %
                         wrap_params.bitmap_size_y;
          }
          CellCoord neighbor = {neighbor_x, neighbor_y};

          if (bitmap.count(neighbor) && !visited.count(neighbor)) {
            q.push(neighbor);
            visited.insert(neighbor);
          }
        }
      }
    }

    // update the largest component
    if (current_component.size() > largest_component.size())
      largest_component = current_component;
  }

  return largest_component;
}

std::vector<int> extractLargestConnectedComponentFromBitmapSphere(
    const std::unordered_map<CellCoord, std::vector<int>, CellCoordHasher>
        &bitmap,
    const int bitmap_size_x, const int bitmap_size_y) {
  // BFS to find the largest connected component
  std::vector<int> largest_component;
  std::unordered_set<CellCoord, CellCoordHasher> visited;

  for (const auto &[key, indices] : bitmap) {
    if (visited.count(key)) continue;  // already visited cell

    std::vector<int> current_component;
    std::queue<CellCoord> q;
    q.push(key);
    visited.insert(key);

    while (!q.empty()) {
      CellCoord current = q.front();
      q.pop();

      // add the current cell to the current component
      auto it = bitmap.find(current);
      if (it != bitmap.end()) {
        current_component.insert(current_component.end(), it->second.begin(),
                                 it->second.end());
      }

      // 8-neighbors exploration
      for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
          if (dx == 0 && dy == 0) continue;  // skip the current cell

          int neighbor_x = current.x + dx;
          int neighbor_y = current.y + dy;

          // handle wrapping, ie periodic boundary conditions
          neighbor_x = (neighbor_x + bitmap_size_x) % bitmap_size_x;

          if (neighbor_y == -1) {
            neighbor_y = 0;
            neighbor_x = bitmap_size_x - neighbor_x - 1;
          } else if (neighbor_y == bitmap_size_y) {
            neighbor_y = bitmap_size_y - 1;
            neighbor_x = bitmap_size_x - neighbor_x - 1;
          }

          CellCoord neighbor = {neighbor_x, neighbor_y};

          if (bitmap.count(neighbor) && !visited.count(neighbor)) {
            q.push(neighbor);
            visited.insert(neighbor);
          }
        }
      }
    }

    // update the largest component
    if (current_component.size() > largest_component.size())
      largest_component = current_component;
  }

  return largest_component;
}

}  // namespace efficient_ransac
