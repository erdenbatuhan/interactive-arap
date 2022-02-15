# ===============================================================================
# Project: Interactive ARAP
# File:    CMakeLists.txt
# Authors: Batuhan Erden, Cansu Yildirim, Anas Shahzad, Alexander Epple
# ===============================================================================

cmake_minimum_required(VERSION 3.13)

project(interactive_arap)

# Always run in release mode
set(CMAKE_BUILD_TYPE Release)

# Set the library directory for additional libraries
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs CACHE PATH "Path to the library folder containing additional libraries")

# Dependency: libigl
list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/libigl)
option(LIBIGL_WITH_OPENGL "Use OpenGL" ON)
option(LIBIGL_WITH_OPENGL_GLFW "Use GLFW" ON)
include(libigl)

# Set header and source files
file(GLOB HEADER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h) # Set header files
file(GLOB SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp) # Set source files

# Add executable and link it to the libraries
add_executable(${PROJECT_NAME} main.cpp ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC igl::core igl::opengl_glfw)

# Additional dependency: ceres (Currently not being used!)
#set(glog_DIR ${LIBRARY_DIR}/glog/lib/cmake/glog CACHE PATH "Path to glog")
#set(Ceres_DIR ${LIBRARY_DIR}/ceres/cmake CACHE PATH "Path to Ceres")
#find_package(Ceres REQUIRED)
#target_link_libraries(${PROJECT_NAME} PUBLIC ceres)

# Optional dependency: OpenMP
find_package(OpenMP)
if(OpenMP_CXX_FOUND) # Link target to OpenMP if it is found
    message("-- OpenMP found! The project will now run in parallel..")

    target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
    add_definitions(-DOMP)
endif()

# Copy resources (models & shaders)
add_custom_command(TARGET ${PROJECT_NAME}
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${CMAKE_CURRENT_SOURCE_DIR}/resources" "${CMAKE_BINARY_DIR}/resources"
    VERBATIM)

# Add additional definitions
add_definitions("-D_DISABLE_EXTENDED_ALIGNED_STORAGE")

# OS-specific configurations
if(WIN32) # Windows
    message("-- Operating system: Windows")

    target_compile_definitions(${PROJECT_NAME} PUBLIC NOMINMAX _USE_MATH_DEFINES)
    set_property(TARGET ${PROJECT_NAME} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/)
elseif(UNIX) # Unix-based Operating Systems
    if(APPLE) # macOS
        message("-- Operating system: macOS")
    else() # Linux
        message("-- Operating system: Linux")
    endif(APPLE)
endif(WIN32)

