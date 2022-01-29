cmake_minimum_required(VERSION 3.13)

project(interactive_arap)

set(CMAKE_CXX_STANDARD 17)

# Make sure OpenGL is installed!
find_package(OpenGL REQUIRED)

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

# Set header and source files
file(GLOB HEADER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/**/*.h) # Set header files
file(GLOB SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/**/*.c*) # Set source files

# Add executable and link it to the libraries
add_executable(interactive_arap main.cpp ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(interactive_arap ceres glfw Eigen3::Eigen glm::glm)

# Add additional definitions
add_definitions("-D_DISABLE_EXTENDED_ALIGNED_STORAGE")

# OS-specific configurations
if (WIN32) # Windows
    message("-- Operating system: Windows")

    target_compile_definitions(interactive_arap PUBLIC NOMINMAX _USE_MATH_DEFINES)
    set_property(TARGET interactive_arap PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/)

    add_custom_command(TARGET interactive_arap POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${glfw3_DIR}/../../../bin/glfw3.dll" "${CMAKE_BINARY_DIR}"
        VERBATIM)
elseif (UNIX) # Unix-based Operating Systems (referenced from https://cse.engineering.nyu.edu/cs653/OpenGLCompilationMacLinux8.pdf)
    if (APPLE) # macOS
        message("-- Operating system: macOS")
    else() # Linux
        message("-- Operating system: Linux")
    endif(APPLE)
endif(WIN32)

