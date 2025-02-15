#pragma once

#include <pcl/point_types.h>

namespace efficient_ransac {

pcl::PointXYZ operator-(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2);

pcl::Normal cross(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2);

void normalize(pcl::Normal &n);

float dot(const pcl::Normal &n1, const pcl::Normal &n2);
float dot(const pcl::Normal &n, const pcl::PointXYZ &p);

}  // namespace efficient_ransac
