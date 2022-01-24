cmake_minimum_required(VERSION 3.13)

project(interactive_arap)

set(CMAKE_CXX_STANDARD 17)

# Set the directory of each library
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs CACHE PATH "Path to the library folder containing additional libraries")
set(Eigen3_DIR ${LIBRARY_DIR}/eigen/share/eigen3/cmake CACHE PATH "Path to Eigen")
set(glog_DIR ${LIBRARY_DIR}/glog/lib/cmake/glog CACHE PATH "Path to glog")
set(Ceres_DIR ${LIBRARY_DIR}/ceres/cmake CACHE PATH "Path to Ceres")
set(glfw3_DIR ${LIBRARY_DIR}/glfw/lib/cmake/glfw3 CACHE PATH "Path to glfw")
set(glm_DIR ${LIBRARY_DIR}/glm/cmake/glm CACHE PATH "Path to glm")

# Find the required packages
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(Ceres REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)

# Additional GLOG options
get_target_property(GLOG_DLL_PATH_DEBUG glog::glog IMPORTED_LOCATION_DEBUG)
get_target_property(GLOG_DLL_PATH_RELEASE glog::glog IMPORTED_LOCATION_RELEASE)

# Add definitions
add_definitions("-D_DISABLE_EXTENDED_ALIGNED_STORAGE")

# Set header files
set(HEADER_FILES
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

# Set source files
set(SOURCE_FILES
    utils/shaderClass.cpp
    utils/stb.cpp
)

# Add executable
add_executable(interactive_arap main.cpp ${HEADER_FILES} ${SOURCE_FILES})

# Link and include directories
target_link_libraries(interactive_arap ceres glfw Eigen3::Eigen glm::glm)

# On Windows, copy DLLs to executable folder
if(WIN32)
    target_compile_definitions(interactive_arap PUBLIC NOMINMAX _USE_MATH_DEFINES)
    set_property(TARGET interactive_arap PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/)

    add_custom_command(TARGET interactive_arap POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${glfw3_DIR}/../../../bin/glfw3.dll" "${CMAKE_BINARY_DIR}"
            VERBATIM)
endif(WIN32)

