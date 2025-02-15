#include "pcl_operations.h"

namespace efficient_ransac {

pcl::PointXYZ operator-(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2) {
  return pcl::PointXYZ(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
}

pcl::Normal cross(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2) {
  return pcl::Normal(p1.y * p2.z - p1.z * p2.y, p1.z * p2.x - p1.x * p2.z,
                     p1.x * p2.y - p1.y * p2.x);
}

void normalize(pcl::Normal &n) {
  double norm = sqrt(n.normal_x * n.normal_x + n.normal_y * n.normal_y +
                     n.normal_z * n.normal_z);
  n.normal_x /= norm;
  n.normal_y /= norm;
  n.normal_z /= norm;
}

float dot(const pcl::Normal &n1, const pcl::Normal &n2) {
  return n1.normal_x * n2.normal_x + n1.normal_y * n2.normal_y +
         n1.normal_z * n2.normal_z;
}

float dot(const pcl::Normal &n, const pcl::PointXYZ &p) {
  return n.normal_x * p.x + n.normal_y * p.y + n.normal_z * p.z;
}

}  // namespace efficient_ransac
