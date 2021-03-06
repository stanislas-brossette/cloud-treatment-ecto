# generate headers.
config_files(dirs.hh)

include_directories(${CMAKE_CURRENT_BINARY_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${rosbuild_msgs_INCLUDE_DIRS}
                    ${geometry_msgs_INCLUDE_DIRS}
                    ${sensor_msgs_INCLUDE_DIRS}
                    ${roscpp_INCLUDE_DIRS})

ectomodule(cloud_treatment DESTINATION cloud_treatment_ecto INSTALL
    cpp/cloud_treatment.cpp
    cpp/cloudviewercell.cpp
    cpp/editheadercell.cpp
    cpp/extractheadercell.cpp
    cpp/normalestimationcell.cpp
    cpp/normalsegmentationcell.cpp
#    cpp/organizedeuclidiansegmentationcell.cpp
    cpp/organizedmultiplanesegmentationcell.cpp
    cpp/passthroughcell.cpp
    cpp/passthrough3dcell.cpp
    cpp/pcdreadercell.cpp
    cpp/principalcomponentextractioncell.cpp
    cpp/rectanglespubcell.cpp
    cpp/regiongrowingcell.cpp
    cpp/recalibratemsgcell.cpp
    cpp/stepcenteringcell.cpp
    cpp/stepsegmentationcell.cpp
    cpp/typedefs.h
    cpp/voxelgridcell.cpp
    cpp/xyzswitchcell.cpp
)

link_ecto(cloud_treatment
  ${PCL_LIBRARIES}
  ${roscpp_LIBRARIES}
)
INSTALL(FILES cloud_treatment_ecto/__init__.py
  DESTINATION "${PYTHON_INSTALL_DIR}/cloud_treatment_ecto")
