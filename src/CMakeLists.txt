# generate headers.
config_files(dirs.hh)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

ectomodule(cloud_treatment DESTINATION cloud_treatment_ecto INSTALL
    cpp/cloud_treatment.cpp
    cpp/cloudviewercell.cpp
    cpp/normalestimationcell.cpp
#    cpp/organizedeuclidiansegmentationcell.cpp
    cpp/organizedmultiplanesegmentationcell.cpp
    cpp/passthroughcell.cpp
    cpp/pcdreadercell.cpp
    cpp/regiongrowingcell.cpp
    cpp/typedefs.h
    cpp/xyzswitchcell.cpp
)

link_ecto(cloud_treatment
  ${PCL_LIBRARIES}
)
INSTALL(FILES cloud_treatment_ecto/__init__.py
  DESTINATION "${PYTHON_INSTALL_DIR}/cloud_treatment_ecto")
