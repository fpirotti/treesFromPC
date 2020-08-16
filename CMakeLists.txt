cmake_minimum_required(VERSION 3.10)
project(treesFromPC)

find_package(PCL 1.9 REQUIRED)
include_directories(include include/LASzip/include include/LASzip/dll)

#file(GLOB_RECURSE files src/*.c src/*.cpp include/*.h include/*.hpp )


add_library (laszip SHARED ${CMAKE_SOURCE_DIR}/LASzip/dll/laszip_api.c)

add_executable(treesFromPC main.cpp)


target_link_libraries(treesFromPC /usr/lib/x86_64-linux-gnu/liblaszip.so laszip ${PCL_LIBRARIES})
set(CMAKE_SHARED_LINKER_FLAGS CACHE FORCE "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--as-needed -Wl,--no-undefined -Wl,-z,now -Wl,--no-as-needed -ldl")
#    message(WARNING  "${LASZIP_INCLUDE_DIRS} DO NOT run git clone https://github.com/LASzip/LASzip inside this directory!" )

