#include <ecto/ecto.hpp>

//Defines a top level ecto python cell with the name 'cloud_treatment_ecto'
//Note that the shared library must be called cloud_treatment_ecto.so
//In CMake this is achieved using the ecto specific macro:
//ectomodule(cloud_treatment_ecto INSTALL
//                    DESTINATION whatever_folder
//                    cloud_treatment_ecto.cpp
//                    <source1.cpp>
//                    <source2.cpp>
//)

ECTO_DEFINE_MODULE(cloud_treatment) { }
