cmake_minimum_required(VERSION 3.13)

project(interactive_arap)

set(CMAKE_CXX_STANDARD 17)

# Set the directory of each library
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs CACHE PATH "Path to the library folder containing additional libraries")
set(Eigen3_DIR ${LIBRARY_DIR}/eigen/share/eigen3/cmake CACHE PATH "Path to Eigen")
set(glog_DIR ${LIBRARY_DIR}/glog/lib/cmake/glog CACHE PATH "Path to glog")
set(Ceres_DIR ${LIBRARY_DIR}/ceres/cmake CACHE PATH "Path to Ceres")
set(glfw3_DIR ${LIBRARY_DIR}/glfw/lib/cmake/glfw3 CACHE PATH "Path to glfw")
set(GLEW_ROOT ${LIBRARY_DIR}/glew CACHE PATH "Path to GLEW")
set(glm_DIR ${LIBRARY_DIR}/glm CACHE PATH "Path to glm")

# Find the required packages
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(Ceres REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)
find_package(OpenGL REQUIRED)

# Additional GLOG options
get_target_property(GLOG_DLL_PATH_DEBUG glog::glog IMPORTED_LOCATION_DEBUG)
get_target_property(GLOG_DLL_PATH_RELEASE glog::glog IMPORTED_LOCATION_RELEASE)

file(GLOB HEADER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h) # Set header files
file(GLOB SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c*) # Set source files

# Add executable and link it to the libraries
add_executable(interactive_arap main.cpp ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(interactive_arap ceres glfw Eigen3::Eigen glm::glm GLEW::glew OpenGL::GL)

# Add additional definitions
add_definitions("-D_DISABLE_EXTENDED_ALIGNED_STORAGE")

# Copy resources & shaders
add_custom_command(TARGET interactive_arap
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders" "${CMAKE_BINARY_DIR}/shaders"
        VERBATIM)
add_custom_command(TARGET interactive_arap
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/resources" "${CMAKE_BINARY_DIR}/resources"
        VERBATIM)

# OS-specific configurations
if (WIN32) # Windows
    message("-- Operating system: Windows")

# On Windows, copy DLLs to executable folder
if(WIN32)
    target_compile_definitions(interactive_arap PUBLIC NOMINMAX _USE_MATH_DEFINES)
    set_property(TARGET interactive_arap PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/)

    add_custom_command(TARGET interactive_arap POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${glfw3_DIR}/../../../bin/glfw3.dll" "${CMAKE_BINARY_DIR}"
        VERBATIM)
    add_custom_command(TARGET interactive_arap POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${GLEW_ROOT}/bin/Release/x64/glew32.dll" "${CMAKE_BINARY_DIR}"
        VERBATIM)
elseif (UNIX) # Unix-based Operating Systems (referenced from https://cse.engineering.nyu.edu/cs653/OpenGLCompilationMacLinux8.pdf)
    if (APPLE) # macOS
        message("-- Operating system: macOS")
    else() # Linux
        message("-- Operating system: Linux")
    endif(APPLE)

    # Find & add GLUT - may not be necessary!
    find_package(GLUT REQUIRED)
    target_link_libraries(interactive_arap GLUT::GLUT)

    if (APPLE) # macOS
        # Suppress warnings of the deprecation of glut functions on macOS.
        add_definitions(-Wno-deprecated-declarations)
    endif(APPLE)

    # Add the list of include paths to be used to search for include files
    include_directories(interactive_arap ${UNIX_INCLUDE_DIRS})

    # Link the executable to the libraries
    target_link_libraries(interactive_arap ${UNIX_LIBRARIES})
endif(WIN32)

