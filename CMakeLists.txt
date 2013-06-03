
cmake_minimum_required(VERSION 2.8)

project(cloud_treatment_ecto)

find_package(catkin REQUIRED)
find_package(ecto REQUIRED)
find_package(ecto_pcl REQUIRED)
find_package(PCL 1.7 REQUIRED
	COMPONENTS apps common io surface filters segmentation visualization)

# Use PCL
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

ectomodule(cloud_treatment DESTINATION cloud_treatment_ecto INSTALL
    src/cpp/cloud_treatment.cpp
    src/cpp/cloudviewercell.cpp
    src/cpp/passthroughcell.cpp
    src/cpp/pcdreadercell.cpp
    src/cpp/xyzswitchcell.cpp
)

link_ecto(cloud_treatment
  ${PCL_LIBRARIES}
)

add_subdirectory(bin)
add_subdirectory(share)
add_subdirectory(src)
add_subdirectory(test)
add_subdirectory(var)
