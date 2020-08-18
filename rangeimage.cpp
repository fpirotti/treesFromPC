#include "rangeimage.h"

rangeImage::rangeImage()
{

}


// --------------
// -----Help-----
// --------------
void
  rangeImage::printUsage (const char* progName)
  {
    std::cout << "\n\nUsage: "<<progName<<" [options] <scene.pcd>\n\n"
              << "Options:\n"
              << "-------------------------------------------\n"
              << "-rx <float>  angular resolution in degrees (default "<<angular_resolution_x<<")\n"
              << "-ry <float>  angular resolution in degrees (default "<<angular_resolution_y<<")\n"
              << "-c <int>     coordinate frame (default "<< (int)coordinate_frame<<")\n"
              << "-l           live update - update the range image according to the selected view in the 3D viewer.\n"
              << "-h           this help\n"
              << "\n\n";
  }

void
  rangeImage::setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
  {
    Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f(0, 0, 1) + pos_vector;
    Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f(0, -1, 0);
    viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
                              look_at_vector[0], look_at_vector[1], look_at_vector[2],
                                                                                  up_vector[0], up_vector[1], up_vector[2]);
  }


pcl::PointCloud<pcl::PointXYZ>::Ptr rangeImage::passThroughFilter1D(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string field, const double low, const double high, const bool remove_inside)
{
    if (low > high)
    {
        std::cout << "Warning! Min is greater than max!\n";
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;

    pass.setInputCloud(cloud);
    pass.setFilterFieldName(field);
    pass.setFilterLimits(low, high);
    pass.setFilterLimitsNegative(remove_inside);
    pass.filter(*cloud_filtered);
    return cloud_filtered;
}



void
  rangeImage::view(const pcl::PointCloud<pcl::PointXYZ>::Ptr  &cloud_ptr,
                   const Eigen::Vector3f& point_cloud_center,
                   float point_cloud_radius)
  {
    angular_resolution_x = pcl::deg2rad (angular_resolution_x);
    angular_resolution_y = pcl::deg2rad (angular_resolution_y);

    // ------------------------------------------------------------------
    // -----Read pcd file or create example point cloud if not given-----
    // ------------------------------------------------------------------

//    Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
   // std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");


    // -----------------------------------------------
    // -----Create RangeImage from the PointCloud-----
    // -----------------------------------------------
//    float noise_level = 0.0;
//    float min_range = 0.0f;
//    int border_size = 1;
    //pcl::RangeImage::Ptr range_image_ptr(new pcl::RangeImage);
    //pcl::RangeImage& range_image = *range_image_ptr;

//    range_image.createFromPointCloud ( *cloud,
//                                      angular_resolution_x,
//                                      angular_resolution_y,
//                                      pcl::deg2rad (360.0f),
//                                      pcl::deg2rad (180.0f),
//                                      scene_sensor_pose,
//                                      coordinate_frame,
//                                      noise_level,
//                                      min_range,
//                                      border_size);

    pcl::PointCloud<pcl::PointXYZ> point_cloud;

    point_cloud = *cloud_ptr ;
//    range_image.createFromPointCloudWithKnownSize ( point_cloud,
//                                                    angular_resolution_x,
//                                                    angular_resolution_y,
//                                                    point_cloud_center,
//                                                    point_cloud_radius);
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------


    pcl::visualization::PCLVisualizer viewer ("3D Viewer");
    viewer.setBackgroundColor (1, 1, 1);
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0, 0, 0);
   // viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
    //viewer.addCoordinateSystem (1.0f, "global");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> point_cloud_color_handler (cloud_ptr, 150, 150, 150);
    viewer.addPointCloud (cloud_ptr, "PC");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "PC");
    viewer.initCameraParameters ();
    //setViewerPose(viewer, range_image.getTransformationToWorldSystem ());

    // --------------------------
    // -----Show range image-----
    // --------------------------
    //pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
    // range_image_widget.showRangeImage (range_image);

    //--------------------
    // -----Main loop-----
    //--------------------
    while (!viewer.wasStopped ())
    {

       viewer.spinOnce ();
      pcl_sleep (0.01);

//      if (live_update)
//      {
//        scene_sensor_pose = viewer.getViewerPose();
//        range_image.createFromPointCloud (point_cloud, angular_resolution_x, angular_resolution_y,
//                                          pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
//                                          scene_sensor_pose, pcl::RangeImage::LASER_FRAME, noise_level, min_range, border_size);
//        range_image_widget.showRangeImage (range_image);
//      }
    }
  }

