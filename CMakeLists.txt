cmake_minimum_required(VERSION 2.8)

project(create_coalpile)
# pcl
find_package( PCL REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

add_executable(main "main.cpp")
target_link_libraries(main ${PCL_LIBRARIES})
