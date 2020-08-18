#ifndef RANGEIMAGE_H
#define RANGEIMAGE_H

#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>


#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>


class rangeImage
{
public:
    //pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr;
    // --------------------
    // -----Parameters-----
    // --------------------
    float angular_resolution_x = 1.5f,
      angular_resolution_y = angular_resolution_x;
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    bool live_update = false;
    pcl::PointCloud<pcl::PointXYZ>::Ptr passThroughFilter1D(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string field, const double low, const double high, const bool remove_inside);

    rangeImage();
    void setPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud);
    void view(const pcl::PointCloud<pcl::PointXYZ>::Ptr  &cloud_ptr, const Eigen::Vector3f& point_cloud_center, float point_cloud_radius);
    void setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose);
    void  printUsage (const char* progName);
};

#endif // RANGEIMAGE_H
