#include "shape.h"

namespace efficient_ransac {

void Shape::removeFromInliersIndices(std::vector<int> indices_to_remove) {
    inliers_indices_.erase(std::remove_if(inliers_indices_.begin(), inliers_indices_.end(),
        [&indices_to_remove](int inlier) {
            return std::find(indices_to_remove.begin(), indices_to_remove.end(),
                inlier) != indices_to_remove.end();
        }), inliers_indices_.end());
}

void Shape::computeInliersIndices(
    const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud,
    const std::shared_ptr<pcl::octree::OctreePointCloudSearch<pcl::PointNormal>> &octree,
    const std::vector<bool> &remaining_points,
    const bool compute_connected_components) {

    inliers_indices_.clear();

    std::vector<int> indices_close_to_shape;
    getIndicesCloseToShape(octree, remaining_points, indices_close_to_shape);
    for (auto i : indices_close_to_shape) {
        pcl::PointNormal point = cloud->at(i);
        if (distance(point)<thresholds_.distance && angle(point)<thresholds_.normal) {
        inliers_indices_.push_back(i);
        }
    } 
  // extract the largest connected component
  if (compute_connected_components) 
    extractLargestConnectedComponent(cloud);
}

void Shape::getIndicesCloseToShape(
    const std::shared_ptr<pcl::octree::OctreePointCloudSearch<pcl::PointNormal>> &octree,
    const std::vector<bool> &remaining_points,
    std::vector<int> &indices_close_to_shape){  
    const float resolution = octree->getResolution();
    for (auto it = octree->leaf_depth_begin(); it != octree->leaf_depth_end(); ++it) {
        Eigen::Vector3f min_pt, max_pt;
        octree->getVoxelBounds(it, min_pt, max_pt);
        Eigen::Vector3f voxel_center = (min_pt + max_pt) / 2.0;
        pcl::PointNormal center(voxel_center[0], voxel_center[1], voxel_center[2]);
        float dist_voxel_shape=distance(center);
        // Check if the voxel is close to the shape
        if (pow(dist_voxel_shape-thresholds_.distance,2)<pow(resolution,2)*3.0/4){
            std::vector<int> point_indices_voxel;
            it.getLeafContainer().getPointIndices(point_indices_voxel);
            for (auto idx : point_indices_voxel) {
                if (remaining_points[idx]) indices_close_to_shape.push_back(idx);
            }
        }
    }
}

void Shape::getIndicesCloseToShapeSmall(
    const std::shared_ptr<pcl::octree::OctreePointCloudSearch<pcl::PointNormal>> &small_octree,
    const std::vector<bool> &remaining_points,
    const std::vector<int> &subsampled_indices,
    std::vector<int> &indices_close_to_shape){  
    const float resolution = small_octree->getResolution();
    for (auto it = small_octree->leaf_depth_begin(); it != small_octree->leaf_depth_end(); ++it) {
        Eigen::Vector3f min_pt, max_pt;
        small_octree->getVoxelBounds(it, min_pt, max_pt);
        Eigen::Vector3f voxel_center = (min_pt + max_pt) / 2.0;
        pcl::PointNormal center(voxel_center[0], voxel_center[1], voxel_center[2]);
        float dist_voxel_shape=distance(center);
        // Check if the voxel is close to the shape
        if (pow(dist_voxel_shape-thresholds_.distance,2)<pow(resolution,2)*3.0/4){
            std::vector<int> point_indices_voxel;
            it.getLeafContainer().getPointIndices(point_indices_voxel);
            for (auto idx : point_indices_voxel) {
                if (remaining_points[subsampled_indices[idx]]) indices_close_to_shape.push_back(idx);
            }
        }
    }
}

void Shape::computeExpectationScore(
    const std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> &cloud,
    const std::shared_ptr<pcl::octree::OctreePointCloudSearch<pcl::PointNormal>> &small_octree,
    const std::vector<bool> &remaining_points,
    const std::vector<int> &subsampled_indices){
        expected_score_=0;
        std::vector<int> indices_close_to_shape;
        getIndicesCloseToShapeSmall(small_octree, remaining_points, subsampled_indices, indices_close_to_shape);
        for (auto i : indices_close_to_shape) {
            pcl::PointNormal point = cloud->at(subsampled_indices[i]);
            if (distance(point)<thresholds_.distance && angle(point)<thresholds_.normal) {
            expected_score_++;
            }
        } 
        const int cloud_size = std::count(remaining_points.begin(), remaining_points.end(), true);
        // subsample indices where remaining_points is true
        const int small_cloud_size = std::count_if(subsampled_indices.begin(), subsampled_indices.end(), [&remaining_points](int index) {
            return remaining_points[index];
        });
        expected_score_= (2+ cloud_size)*(1+expected_score_)/(2+small_cloud_size)-1;
    }


    


}  // namespace efficient_ransac


