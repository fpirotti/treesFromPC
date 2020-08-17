#include <iostream>
#include <helpers.h>

#if defined(_MSC_VER) && \
    (_MSC_FULL_VER >= 150000000)
#define LASCopyString _strdup
#else
#define LASCopyString strdup
#endif

#include <fstream>  // std::ofstream
#include <algorithm> // std::copy
#include <exception> // std::exception

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

using namespace std;

laszip_U8 version_major;
laszip_U8 version_minor;
laszip_U16 version_revision;
laszip_U32 version_build;

laszip_POINTER laszip_reader;
laszip_header* header;
laszip_point* point;
int colorScale;
double coordinates[3];
double refCoordinates[3];


int main(int argc, char *argv[])
{

    pcl::PCDReader reader;
    pcl::PCDWriter writer;
    std::stringstream ss;
    std::string  path;

    //
    if(argc>0){
         path = std::string(argv[1]);
    }
    else {
        return -1;
    }

    if (laszip_load_dll())
    {
      fprintf(stderr,"DLL ERROR: loading LASzip DLL\n");
      return -1;
    }

    // get version of LASzip DLL

    if (laszip_get_version(&version_major, &version_minor, &version_revision, &version_build))
    {
      fprintf(stderr,"DLL ERROR: getting LASzip DLL version number\n");
      return -1;
    }

    fprintf(stderr,"LASzip DLL v%d.%d r%d (build %d)\n", (int)version_major, (int)version_minor, (int)version_revision, (int)version_build);


    laszip_create(&laszip_reader);

    laszip_BOOL request_reader = 1;
    laszip_request_compatibility_mode(laszip_reader, request_reader);
    laszip_BOOL is_compressed = iEndsWith(path, ".laz") ? 1 : 0;
    laszip_open_reader(laszip_reader, path.c_str(), &is_compressed);

    laszip_get_header_pointer(laszip_reader, &header);

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
    laszip_get_coordinates(laszip_reader, refCoordinates);

    for (std::size_t i = 0; i < cloud->size (); ++i)
    {

        laszip_get_coordinates(laszip_reader, coordinates);

        (*cloud)[i].x = (float) (coordinates[0] - refCoordinates[0]);
        (*cloud)[i].y = (float) (coordinates[1] - refCoordinates[1]);
        (*cloud)[i].z = (float) (coordinates[2] - refCoordinates[2]);

        laszip_read_point(laszip_reader);

       // fprintf(stderr,"1 - x%f y%f z%f )\n" , (*cloud)[i].x, (*cloud)[i].y, (*cloud)[i].z);
    }


    float resolution = 0.5f;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);

    octree.setInputCloud (cloud);
    octree.addPointsFromInputCloud ();

    std::cerr << "coords: " << npoints << " " <<  std::endl;
    laszip_close_reader(laszip_reader);


    current_time = time(NULL) - current_time;

    cout << "SEconds:   " <<  current_time << endl;

    current_time = time(NULL);

    int leafNodeCounter =0;
    for (auto it1 = octree.leaf_depth_begin(), it1_end = octree.leaf_depth_end(); it1 != it1_end; ++it1)
    {

      size_t size = it1.getLeafContainer().getSize();
      cout << "Size leaf :   " <<  size << endl;
      leafNodeCounter++;
    }

    current_time = time(NULL) - current_time;
    cout << "SEconds:   " <<  current_time << endl;



    return 0;
}

void help(){

}


int las2pcl()
{
 return 0;
}
