#ifndef OCTREE_H
#define OCTREE_H


#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>

#include <iostream>
#include <vector>
#include <ctime>

class octree
{
public:
    float resolution = 128.0f;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    octree();
};

#endif // OCTREE_H
