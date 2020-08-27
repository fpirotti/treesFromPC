#ifndef HELPERS_H
#define HELPERS_H

#include <rangeimage.h>
#include <laszip/laszip_api.h>

#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>

#include <ctime>
#include <vector>
#include <map>
#include <iostream>
#include <math.h>
#include <string>
#include <fstream>
#include <algorithm>

//#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>

using namespace std;

using std::ifstream;
using std::ofstream;
using std::ios;
using std::string;
using std::min;
using std::max;
using std::ostream;
using std::cout;
using std::cin;
using std::endl;
using std::vector;
using std::map;

laszip_U8 version_major;
laszip_U8 version_minor;
laszip_U16 version_revision;
laszip_U32 version_build;

laszip_POINTER laszip_reader, laszip_writer;
laszip_header* header;
laszip_point* point;
int colorScale;
double coordinates[3];
double refCoordinates[3];


void log(string message, int type, bool exit) {
    if (type == 0) pcl::console::print_error(message.c_str());
    else if (type == 1) pcl::console::print_warn(message.c_str());
    else  pcl::console::print_info(message.c_str());

#ifdef _WIN32
    if (exit) { system("pause"); }
#endif

};







/**
 * from http://stackoverflow.com/questions/5840148/how-can-i-get-a-files-size-in-c
 */
long filesize(string filename) {
    struct stat stat_buf;
    int rc = stat(filename.c_str(), &stat_buf);
    return rc == 0 ? stat_buf.st_size : -1;
};


///**
// * from http://stackoverflow.com/questions/874134/find-if-string-endswith-another-string-in-c
// */
//bool endsWith (std::string const &fullString, std::string const &ending)
//{
//    if (fullString.length() >= ending.length()) {
//        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
//    } else {
//        return false;
//    }
//}

/**
 * see http://stackoverflow.com/questions/735204/convert-a-string-in-c-to-upper-case
 */
string toUpper(string str){
    string tmp = str;
    std::transform(tmp.begin(), tmp.end(),tmp.begin(), ::toupper);

    return tmp;
}


float psign(float value){
    if(value == 0.0){
        return 0.0;
    }else if(value < 0.0){
        return -1.0;
    }else{
        return 1.0;
    }
}


// see https://stackoverflow.com/questions/23943728/case-insensitive-standard-string-comparison-in-c
bool icompare_pred(unsigned char a, unsigned char b) {
    return std::tolower(a) == std::tolower(b);
}

// see https://stackoverflow.com/questions/23943728/case-insensitive-standard-string-comparison-in-c
bool icompare(std::string const& a, std::string const& b) {
    if (a.length() == b.length()) {
        return std::equal(b.begin(), b.end(), a.begin(), icompare_pred);
    }
    else {
        return false;
    }
}

//bool endsWith(const std::string &str, const std::string &suffix) {
//	return str.size() >= suffix.size() && str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
//}

bool endsWith(const string &str, const string &suffix) {

    if (str.size() < suffix.size()) {
        return false;
    }

    auto tstr = str.substr(str.size() - suffix.size());

    return tstr.compare(suffix) == 0;
}

bool iEndsWith(const std::string &str, const std::string &suffix) {

    if (str.size() < suffix.size()) {
        return false;
    }

    auto tstr = str.substr(str.size() - suffix.size());

    return icompare(tstr, suffix);
}

vector<string> splita(string str, vector<char> delimiters) {

    vector<string> tokens;

    auto isDelimiter = [&delimiters](char ch) {
        for (auto &delimiter : delimiters) {
            if (ch == delimiter) {
                return true;
            }
        }

        return false;
    };

    int start = 0;
    for (int i = 0; i < str.size(); i++) {
        if (isDelimiter(str[i])) {
            if (start < i) {
                auto token = str.substr(start, i - start);
                tokens.push_back(token);
            }

            start = i + 1;
        }
    }

    if (start < str.size()) {
        tokens.push_back(str.substr(start));
    }

    return tokens;
}

string  aggregate(vector<string> v, string i="") {
    std::string s;
    for (const auto &piece : v) {
        s += piece;
        if( i !="") s += i;
    }
    return s;
}


vector<string> split(string str, char delimiter) {
    return splita(str, { delimiter });
}

// see https://stackoverflow.com/questions/216823/whats-the-best-way-to-trim-stdstring
string ltrim(string s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
        return !std::isspace(ch);
    }));

    return s;
}

// see https://stackoverflow.com/questions/216823/whats-the-best-way-to-trim-stdstring
string rtrim(string s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
        return !std::isspace(ch);
    }).base(), s.end());

    return s;
}

// see https://stackoverflow.com/questions/216823/whats-the-best-way-to-trim-stdstring
string trim(string s) {
    s = ltrim(s);
    s = rtrim(s);

    return s;
}



static void dll_error(laszip_POINTER laszip)
{
  if (laszip)
  {
    laszip_CHAR* error;
    if (laszip_get_error(laszip, &error))
    {
      fprintf(stderr,"DLL ERROR: getting error messages\n");
    }
    fprintf(stderr,"DLL ERROR MESSAGE: %s\n", error);
  }
}




void openAllLAS(std::string& path) {

    std::string outpath;
    pcl::console::print_error("LAS/LAZ file: %s\n", path.c_str());
    vector<string> ss = split(path, '.');
    ss.pop_back();
    outpath = aggregate(ss).append("_out.laz");
    pcl::console::print_error("output LAS/LAZ file: %s\n", outpath.c_str());

    laszip_create(&laszip_reader);
    laszip_create(&laszip_writer);

    laszip_BOOL request_reader = 1;
    laszip_request_compatibility_mode(laszip_reader, request_reader);
    laszip_request_compatibility_mode(laszip_writer, request_reader);
    laszip_BOOL is_compressed = iEndsWith(path, ".laz") ? 1 : 0;
    laszip_BOOL is_compressed_writer = iEndsWith(outpath, ".laz") ? 1 : 0;
    laszip_open_reader(laszip_reader, path.c_str(), &is_compressed);
    laszip_open_writer(laszip_writer, outpath.c_str(), is_compressed_writer);


    if (laszip_get_header_pointer(laszip_reader, &header))
    {
        log("DLL ERROR: getting header pointer from laszip reader\n", 0, true);
        return;
    }
    if (laszip_set_header(laszip_writer, header))
    {
        log("DLL ERROR: getting header pointer from laszip writer\n", 0, true);
        return;
    }



}

void closeAllLAS() {

    if (laszip_close_writer(laszip_writer))
    {

        log("DLL ERROR: closing laszip writer\n", 0, true);
        return;
    }

    // destroy the writer

    if (laszip_destroy(laszip_writer))
    {
        log("DLL ERROR: destroying laszip writer\n", 0, true);
        return;
    }

    // close the reader

    if (laszip_close_reader(laszip_reader))
    {
        log("DLL ERROR: closing laszip reader\n", 0, true);
        return;
    }

    // destroy the reader

    if (laszip_destroy(laszip_reader))
    {
        log("DLL ERROR: destroying laszip reader\n", 0, true);
        return;
    }



}


#endif // HELPERS_H
