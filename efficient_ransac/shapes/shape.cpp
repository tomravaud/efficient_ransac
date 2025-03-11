#include "shape.h"

namespace efficient_ransac {

void Shape::removeFromInliersIndices(std::vector<int> indices_to_remove) {
    inliers_indices_.erase(std::remove_if(inliers_indices_.begin(), inliers_indices_.end(),
        [&indices_to_remove](int inlier) {
            return std::find(indices_to_remove.begin(), indices_to_remove.end(),
                inlier) != indices_to_remove.end();
        }), inliers_indices_.end());
}

}  // namespace efficient_ransac


