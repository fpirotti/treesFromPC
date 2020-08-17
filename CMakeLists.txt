cmake_minimum_required(VERSION 3.10)
project(treesFromPC)

find_package(PCL 1.9 REQUIRED)

include_directories(include  ${CMAKE_SOURCE_DIR}/../LASzip/include ${CMAKE_SOURCE_DIR}/../LASzip/dll)

#include_directories(include ${LASZIP_INCLUDE_DIRS})
#file(GLOB_RECURSE files src/*.c src/*.cpp include/*.h include/*.hpp )

add_executable(treesFromPC main.cpp)


target_link_libraries(treesFromPC  ${CMAKE_SOURCE_DIR}/../LASzip/build/lib/liblaszip.so ${CMAKE_SOURCE_DIR}/../LASzip/build/lib/liblaszip_api.so ${PCL_LIBRARIES})
set(CMAKE_SHARED_LINKER_FLAGS CACHE FORCE "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--as-needed -Wl,--no-undefined -Wl,-z,now -Wl,--no-as-needed -ldl")
    message(WARNING  " ${CMAKE_SOURCE_DIR}/../LASzip/build/lib/liblaszip.so  DO NOT run git clone https://github.com/LASzip/LASzip inside this directory!" )

