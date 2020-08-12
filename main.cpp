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

int main(int argc, char *argv[])
{

    pcl::PCDReader reader;
    pcl::PCDWriter writer;
    std::stringstream ss;
    //


    if (laszip_load_dll())
    {
      fprintf(stderr,"DLL ERROR: loading LASzip DLL\n");
    }

    // get version of LASzip DLL

    if (laszip_get_version(&version_major, &version_minor, &version_revision, &version_build))
    {
      fprintf(stderr,"DLL ERROR: getting LASzip DLL version number\n");
    }
    else
    {
      fprintf(stderr,"LASzip DLL v%d.%d r%d (build %d)\n", (int)version_major, (int)version_minor, (int)version_revision, (int)version_build);
    }




    try
    {

    }
    catch (std::exception const& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
    }



    cout << "Hello World!   " <<  argc << endl;
    if(argc>0) cout << " arg is " << argv[1] << endl;
    return 0;
}

void help(){

}


int las2pcl()
{
 return 0;
}
