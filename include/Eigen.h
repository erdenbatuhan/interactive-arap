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

#define EIGEN_USE_BLAS

static inline Eigen::MatrixXd safeReplicate(Eigen::MatrixXd& matrix) {
    Eigen::MatrixXd newMatrix(matrix.rows(), matrix.cols());

    for (unsigned int i = 0; i < matrix.rows(); i++) {
        for (unsigned int j = 0; j < matrix.cols(); j++) {
            newMatrix(i, j) = matrix(i, j);
        }
    }

    return newMatrix;
}

#endif // _EIGEN_H_

