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



/**
 * from http://stackoverflow.com/questions/5840148/how-can-i-get-a-files-size-in-c
 */
long filesize(string filename){
    struct stat stat_buf;
    int rc = stat(filename.c_str(), &stat_buf);
    return rc == 0 ? stat_buf.st_size : -1;
}


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

#endif // HELPERS_H
