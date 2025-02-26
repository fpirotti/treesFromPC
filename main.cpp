#include <iostream>
#include <helpers.h>

#if defined(_MSC_VER) && \
    (_MSC_FULL_VER >= 150000000)
#define LASCopyString _strdup
#else
#define LASCopyString strdup
#endif



 
int main(int argc, char *argv[])
{

    pcl::PCDReader reader;
    pcl::PCDWriter writer;
    std::stringstream ss;
    std::string  path;
    
    if (argc < 2)
    {
        log("Syntax is: %s <las-file> \n "
                                       "..... \n"
                                       "......\n", 0, true);
       
         return (1);
    }
    
    //
    if(argc>0){
         path = std::string(argv[1]);
    }
    else {
        log("??????????????", 0, true);
         return -1;
    }

    if (laszip_load_dll())
    {
        log("DLL ERROR: loading LASzip DLL\n", 0, true);
        return -1;
    }

    // get version of LASzip DLL

    if (laszip_get_version(&version_major, &version_minor, &version_revision, &version_build))
    {
      log("DLL ERROR: getting LASzip DLL version number\n", 0, true);
      return -1;
    }

    fprintf(stderr,"LASzip DLL v%d.%d r%d (build %d)\n", (int)version_major, (int)version_minor, (int)version_revision, (int)version_build);
    
    
    openAllLAS(path);

    long long npoints = (header->number_of_point_records ? header->number_of_point_records : header->extended_number_of_point_records);

    laszip_get_point_pointer(laszip_reader, &point);

 

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // Generate pointcloud data
    cloud->width = npoints;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);
     
    time_t current_time;

    current_time = time(NULL);
    fprintf(stderr,"1 - x%f y%f z%f )\n" , header->x_offset, header->y_offset, header->z_offset );
    fprintf(stderr,"2 - x%f y%f z%f )\n" , header->x_scale_factor, header->y_scale_factor, header->x_scale_factor );

    laszip_read_point(laszip_reader);

    // grab first coordinate and then make all other coordinates relative to that
    laszip_get_coordinates(laszip_reader, refCoordinates);
    
    pcl::console::print_highlight ("Reading %d points from LAS/LAZ cloud...\n", npoints);
    
    Eigen::Vector3f center_eigen;
    double radius [3]= {0,0,0};
    double center [3]= {0,0,0};

    int every = cloud->size () / 100;
    
    for (std::size_t i = 0; i < cloud->size (); ++i)
    {

        if( i%every==0 ) pcl::console::print_highlight ("\r%.1f%%...", (float)(i/every) );
        laszip_get_coordinates(laszip_reader, coordinates);

        (*cloud)[i].x = (float) (coordinates[0] - refCoordinates[0]);
        (*cloud)[i].y = (float) (coordinates[1] - refCoordinates[1]);
        (*cloud)[i].z = (float) (coordinates[2] - refCoordinates[2]);
        coordinates[0]= (*cloud)[i].x;
        coordinates[1]= (*cloud)[i].y;
        coordinates[2]= (*cloud)[i].z;
         
        center[0] += (*cloud)[i].x;
        center[1] += (*cloud)[i].y;
        center[2] += (*cloud)[i].z;

        radius[0] += pow((*cloud)[i].x,2);
        radius[1] += pow((*cloud)[i].y,2);
        radius[2] += pow((*cloud)[i].z,2);


        laszip_read_point(laszip_reader);

        laszip_set_coordinates(laszip_writer, coordinates);
        laszip_set_point(laszip_writer, point);
        laszip_write_point(laszip_writer);

       // fprintf(stderr,"1 - x%f y%f z%f )\n" , (*cloud)[i].x, (*cloud)[i].y, (*cloud)[i].z);
    }


    radius[0] *= (*cloud).size();
    radius[1] *= (*cloud).size();
    radius[2] *= (*cloud).size();

    radius[0] -= pow(center[0] ,2);
    radius[1] -= pow(center[1] ,2);
    radius[2] -= pow(center[2] ,2);

    radius[0] =  sqrt( radius[0]) / (*cloud).size();
    radius[1] =  sqrt( radius[1]) / (*cloud).size();
    radius[2] =  sqrt( radius[2]) / (*cloud).size();

    center[0] /= (*cloud).size();
    center[1] /= (*cloud).size();
    center[2] /= (*cloud).size();

    center_eigen.x()=center[0];
    center_eigen.y()=center[1];
    center_eigen.z()=center[2];

    pcl::console::print_highlight ("Average xyz %2f, %2f, %2f...\n", center[0], center[1], center[2]);
    pcl::console::print_highlight ("Std dev xyz %2f, %2f, %2f...\n", radius[0], radius[1], radius[2]);
    pcl::console::print_highlight ("Loading point cloud...\n");

    pcl::octree::OctreePointCloud<pcl::PointXYZ> octree1m (1.0f); // low resolution
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (0.2f); // high resolution



    octree.setInputCloud (cloud);
    octree1m.setInputCloud (cloud);
    octree.addPointsFromInputCloud ();
    octree1m.addPointsFromInputCloud ();

    std::cerr << "coords: " << npoints << " " <<  std::endl;
    laszip_close_reader(laszip_reader);
    laszip_close_writer(laszip_writer);

    current_time = time(NULL) - current_time;

    cout << "SEconds:   " <<  current_time << endl;

    current_time = time(NULL);

    typedef pcl::PointXYZ PointType;
    pcl::PointCloud<PointType>::Ptr s_point_cloud_ptr (new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>& s_point_cloud = *s_point_cloud_ptr;
    //pcl::octree::OctreePointCloudAdjacency<pcl::PointXYZ>::VoxelAdjacencyList supervoxel_adjacency;
    //std::multimap<std::uint32_t, std::uint32_t> supervoxel_adjacency;
    //octree1m.computeVoxelAdjacencyGraph(supervoxel_adjacency); // (supervoxel_adjacency);
     std::vector<PointType, Eigen::aligned_allocator<PointType> > voxel_center_list_arg;
     double min_x, min_y, min_z, max_x, max_y, max_z;
     octree.getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);

    pcl::octree::OctreeKey key;
    int leafNodeCounter =0;
    for (auto it1 = octree.leaf_depth_begin(), it1_end = octree.leaf_depth_end(); it1 != it1_end; ++it1)
    {

      size_t size = it1.getLeafContainer().getSize();
      key = it1.getCurrentOctreeKey();
      PointType point;
      point.x = static_cast<float>((static_cast<double>(key.x) + 0.5f) * octree.getResolution() +
                                   min_x );
      point.y = static_cast<float>((static_cast<double>(key.y) + 0.5f) * octree.getResolution() +
                                   min_y);
      point.z = static_cast<float>((static_cast<double>(key.z) + 0.5f) * octree.getResolution() +
                                   min_z);

      if(size > 5) s_point_cloud.points.push_back (point);

     // it1.VoxelAdjacencyList;
      //if(size > 5) cout << "Size leaf :   " <<  size << endl;
      leafNodeCounter++;
    }

    current_time = time(NULL) - current_time;
    cout << "SEconds:   " <<  current_time << endl;


    rangeImage *myviewer = new rangeImage();
    myviewer->view(s_point_cloud_ptr, center_eigen, (float)(radius[0]));

    #ifdef _WIN32
        system("pause");
    #endif

 

    return 0;
}

void help(){

}

int las2pcl()
{
 return 0;
}
