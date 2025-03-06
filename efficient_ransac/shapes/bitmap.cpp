#include "bitmap.h"

namespace efficient_ransac {

std::vector<int> extractLargestConnectedComponentFromBitmap(
    const std::unordered_map<CellCoord, std::vector<int>, CellCoordHasher>
        &bitmap) {
  // BFS to find the largest connected component
  std::vector<int> largest_component;
  std::unordered_set<CellCoord, CellCoordHasher> visited;

  for (auto &[key, indices] : bitmap) {
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
          CellCoord neighbor = {current.x + dx, current.y + dy};
          if (bitmap.count(neighbor) && !visited.count(neighbor)) {
            q.push(neighbor);
            visited.insert(neighbor);
          }
        }
      }
    }

    // update the largest component
    if (current_component.size() > largest_component.size()) {
      largest_component = current_component;
    }
  }

  return largest_component;
}

}  // namespace efficient_ransac
