/**
 * Project:      Interactive ARAP
 * File:         Eigen.h
 * Authors:      TUM 3D Scanning & Motion Capture (IN2354) Team (TAs)
 * Contributors: Batuhan Erden, Cansu Yildirim, Anas Shahzad, Alexander Epple
 */

#pragma once

#ifndef _EIGEN_H_
#define _EIGEN_H_

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/NonLinearOptimization>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

static inline void replicate(Eigen::MatrixXd& in, Eigen::MatrixXd& other) {
    for (unsigned int i = 0; i < other.rows(); i++) {
        for (unsigned int j = 0; j < other.cols(); j++) {
            in(i, j) = other(i, j);
        }
    }
}

#endif // _EIGEN_H_

