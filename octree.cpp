#include "octree.h"


octree::octree()
{
  *this->cloud = (*new pcl::PointCloud<pcl::PointXYZ>);
}
