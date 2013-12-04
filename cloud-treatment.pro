OTHER_FILES += \
    CMakeLists.txt \
    bin/CMakeLists.txt \
    share/CMakeLists.txt \
    src/CMakeLists.txt \
    test/CMakeLists.txt \
    var/CMakeLists.txt \
    bin/display_cloud.py \
    bin/testing_algo.py \
    src/dirs.hh.in \
    bin/region_growing.py \
    bin/test_kinect.py \
    bin/test_kinect_opencv.py \
    bin/test_opennicapture.py \
    bin/test_kinect_random.py

SOURCES += \
    src/cpp/cloud_treatment.cpp \
    src/cpp/xyzswitchcell.cpp \
    src/cpp/_blue_print_ecto_pcl_cell.cpp \
    src/cpp/cloudviewercell.cpp \
    src/cpp/passthroughcell.cpp \
    src/cpp/pcdreadercell.cpp \
    src/cpp/regiongrowingcell.cpp \
    src/cpp/normalestimationcell.cpp \
    src/cpp/normalsegmentationcell.cpp \
    src/cpp/organizedmultiplanesegmentationcell.cpp \
    src/cpp/organizedeuclidiansegmentationcell.cpp \
    src/cpp/voxelgridcell.cpp \
    src/cpp/passthrough3dcell.cpp

HEADERS += \
    src/cpp/typedefs.h
