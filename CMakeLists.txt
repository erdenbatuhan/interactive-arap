cmake_minimum_required(VERSION 3.13)

project(arap_proj_1)

# Options
set(CMAKE_CXX_STANDARD 14)
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../Libs CACHE PATH "Path to lib folder")
set(Eigen3_DIR ${LIBRARY_DIR}/Eigen/share/eigen3/cmake CACHE PATH "Path to installed Eigen")
set(glog_DIR ${LIBRARY_DIR}/Glog/lib/cmake/glog CACHE PATH "Path to installed glog")
set(Ceres_DIR ${LIBRARY_DIR}/Ceres/CMake CACHE PATH "Path to installed Ceres")
set(glfw3_DIR ${LIBRARY_DIR}/glfw_ins/lib/cmake/glfw3 CACHE PATH "Path to installed glfw")


find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(Ceres REQUIRED)
find_package(glfw3 REQUIRED)


add_definitions("-D_DISABLE_EXTENDED_ALIGNED_STORAGE")

set(UTILS
    utils/Eigen.h
    utils/io.h
    utils/glad.h
    utils/glad.c
    utils/khrplatform.h
    utils/shaderClass.h
    utils/shaderClass.cpp
    utils/stb.h
    utils/stb.cpp
)



add_executable(main ${UTILS} main.cpp)


target_link_libraries(main ceres glfw)

if(WIN32)
    target_compile_definitions(main PUBLIC NOMINMAX _USE_MATH_DEFINES)

    set_property(TARGET main PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/)
       
    get_target_property(GLOG_DLL_PATH_DEBUG glog::glog IMPORTED_LOCATION_DEBUG)
    get_target_property(GLOG_DLL_PATH_RELEASE glog::glog IMPORTED_LOCATION_RELEASE)
    
endif(WIN32)