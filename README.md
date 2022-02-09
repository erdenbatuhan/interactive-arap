# Interactive ARAP

## As-Rigid-As-Possible Surface Modeling

https://igl.ethz.ch/projects/ARAP/

### Local Environment Setup

Make sure you have **OpenGL** and **CMake GUI** *(for ease of use)* installed on your system! One of the main dependencies, [libigl](https://libigl.github.io/tutorial/), will be installed by itself (*@see* the corresponding cmake module: **./cmake/libigl**).

#### Eigen (Needed only for Ceres as [libigl](https://libigl.github.io/tutorial/) installs its own Eigen distribution)

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

#### GLOG (Needed only for Ceres)

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

#### Ceres

- Create a library directory: `mkdir libs`
- Switch to this directory: `cd libs`
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

