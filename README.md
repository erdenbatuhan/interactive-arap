# Interactive ARAP

## As-Rigid-As-Possible Surface Modeling

https://igl.ethz.ch/projects/ARAP/

### Local Environment Setup

#### Dependencies

Make sure you have **OpenGL**, **GLEW** and **CMake GUI** installed on your system!

#### Prerequisites

- Create a library directory: `mkdir libs`
- Switch to this directory: `cd libs`

Complete the following steps in `libs` directory! Note that normally you would not have to run `make` in the **header-only** libraries, just `make install` would suffice. However, out of completeness, we included both commands in the guidelines below.

#### 1) Eigen

- Clone Eigen: `git clone git@github.com:libigl/eigen.git eigen`
- Switch to eigen directory: `cd eigen`
- Create build directory: `mkdir cmake-build`
- Switch to this directory: `cd cmake-build`
- Set eigen install directory in cmake:
    - `cmake-gui`
    - Set source directory to `<path_to_project>/libs/eigen` and build directory to `<path_to_project>/libs/eigen/cmake-build`.
    - Click "Configure".
    - Set **CMAKE_INSTALL_PREFIX** to `<path_to_project>/libs/eigen` (If you do not see this entry, check the **Advanced** checkbox at the top right.).
    - Click "Generate".
- Then make the Eigen project by running the following commands in `<path_to_project>/libs/eigen/cmake-build`: `make && make install`
- `<path_to_project>/libs/eigen/` folder should contain both `share` and `include`. If not, it means that the Eigen is installed on the system directly. In that case, you can either try the previous step where you set **CMAKE_INSTALL_PREFIX** or directly copy `share` and `include` to `<path_to_project>/libs/eigen/`.

#### 2) GLOG

- Clone GLOG: `git clone https://github.com/google/glog glog`
- Switch to glog directory: `cd glog`
- Checkout to stable version: `git checkout v0.5.0`
- Create build directory: `mkdir cmake-build`
- Switch to this directory: `cd cmake-build`
- Set eigen install directory in cmake:
  - `cmake-gui`
  - Set source directory to `<path_to_project>/libs/glog` and build directory to `<path_to_project>/libs/glog/cmake-build`.
  - Click "Configure".
  - We are not going to use gflags, so unselect the entries **WITH_GFLAGS** and **BUILD_TESTING**.
  - Set **CMAKE_INSTALL_PREFIX** to `<path_to_project>/libs/glog` (If you do not see this entry, check the **Advanced** checkbox at the top right.).
  - Click "Generate".
- Then make the GLOG project by running the following commands in `<path_to_project>/libs/glog/cmake-build`: `make && make install`
- `<path_to_project>/libs/glog/` folder should contain both `lib` and `include`. If not, use the same tips in Eigen guide.

#### 3) Ceres

- Clone Ceres: `git clone git@github.com:ceres-solver/ceres-solver.git ceres`
- Switch to ceres directory: `cd ceres`
- Checkout to stable version: `git checkout 2.0.0`
- Create build directory: `mkdir cmake-build`
- Switch to this directory: `cd cmake-build`
- Set eigen install directory in cmake:
  - `cmake-gui`
  - Set source directory to `<path_to_project>/libs/ceres` and build directory to `<path_to_project>/libs/ceres/cmake-build`.
  - Click "Configure".
  - We are not going to use gflags, so unselect the entries **BUILD_EXAMPLES**, **BUILD_BENCHMARKS**, **BUILD_TESTING**, **GFLAGS**.
  - Ceres uses Eigen, so set **Eigen3_DIR** to `<path_to_project>/libs/eigen`.
  - Ceres uses GLOG, so set **glog_DIR** to `<path_to_project>/libs/glog`.
  - Also deactivate **LAPACK**, **CUSTOM_BLAS** and **SUITESPARSE**.
  - Set **CMAKE_INSTALL_PREFIX** to `<path_to_project>/libs/ceres` (If you do not see this entry, check the **Advanced** checkbox at the top right.).
  - Click "Generate".
- Then make the Ceres project by running the following commands in `<path_to_project>/libs/ceres/cmake-build`: `make && make install`
- `<path_to_project>/libs/ceres/` folder should contain both `cmake` and `include`. If not, use the same tips in Eigen guide.

#### 4) glfw3

- Clone glfw: `git clone git@github.com:glfw/glfw.git glfw`
- Switch to glfw directory: `cd glfw`
- Checkout to stable version: `git checkout 3.3.6`
- Create build directory: `mkdir cmake-build`
- Switch to this directory: `cd cmake-build`
- Set eigen install directory in cmake:
  - `cmake-gui`
  - Set source directory to `<path_to_project>/libs/glfw` and build directory to `<path_to_project>/libs/glfw/cmake-build`.
  - Click "Configure".
  - Set **CMAKE_INSTALL_PREFIX** to `<path_to_project>/libs/glfw` (If you do not see this entry, check the **Advanced** checkbox at the top right.).
  - Click "Generate".
- Then make the glfw project by running the following commands in `<path_to_project>/libs/glfw/cmake-build`: `make && make install`
- `<path_to_project>/libs/glfw/` folder should contain `lib`. If not, use the same tips in Eigen guide.

#### 5) GLEW (Windows only)

- Clone glew: `git clone git@github.com:edoren/glew.git glew`
- Switch to glew directory: `cd glew`
- Checkout to stable version: `git checkout v2.0.0`
- Create build directory: `mkdir build/cmake/cmake-build`
- Switch to this directory: `cd build/cmake/cmake-build`
- Set eigen install directory in cmake:
  - `cmake-gui`
  - Set source directory to `<path_to_project>/libs/glew/build/cmake` and build directory to `<path_to_project>/libs/glew/build/cmake/cmake-build/cmake-build`.
  - Click "Configure".
  - Set **CMAKE_INSTALL_PREFIX** to `<path_to_project>/libs/glew` (If you do not see this entry, check the **Advanced** checkbox at the top right.).
  - Click "Generate".
- Then make the glew project by running the following commands in `<path_to_project>/libs/glew/cmake-build`: `make && make install`
- `<path_to_project>/libs/glew/` folder should contain `lib`. If not, use the same tips in Eigen guide.

#### 6) OpenGL Mathematics (GLM)

- Clone GLM: `git clone git@github.com:g-truc/glm.git glm`
- Switch to GLM directory: `cd glm`
- Checkout to stable version: `git checkout 0.9.9.8`
- You do not need to build this library as it is just an include-only library. Copy the `cd glm` and its contents to `include` folder: `mkdir include && cp -r glm/ include/glm/`

