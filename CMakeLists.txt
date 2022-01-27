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

# TODO: Replace utils with src and remove this TODO comment
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

    # Find OpenGL and GLUT
    find_package(OpenGL REQUIRED)
    find_package(GLUT REQUIRED)

    # CMake built-in variables defined when the packages are found
    set(UNIX_INCLUDE_DIRS ${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})
    set(UNIX_LIBRARIES ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})

    if (APPLE) # macOS
        # Suppress warnings of the deprecation of glut functions on macOS.
        add_definitions(-Wno-deprecated-declarations)
    else() # Linux
        # Find GLEW
        find_package(GLEW REQUIRED)

        # Add glew include directory and library path to lists
        list(APPEND UNIX_INCLUDE_DIRS ${GLEW_INCLUDE_DIRS})
        list(APPEND UNIX_LIBRARIES ${GLEW_LIBRARIES})
    endif(APPLE)

    # Add the list of include paths to be used to search for include files
    include_directories(interactive_arap ${UNIX_INCLUDE_DIRS})

    # Link the executable to the libraries
    target_link_libraries(interactive_arap ${UNIX_LIBRARIES})
endif(WIN32)

