/**
 * Project: Interactive ARAP
 * File:    Arap.h
 * Authors: Batuhan Erden, Cansu Yildirim, Anas Shahzad, Alexander Epple
 */

#ifndef _ARAP_H_
#define _ARAP_H_

#include "Eigen.h"

#include <map>
#include <vector>
#include <utility>

#ifdef OMP
#include <omp.h>

// User-defined reductions (reference: http://www.archer.ac.uk/training/course-material/2018/07/AdvOpenMP-camb/L09-OpenMP4.pdf)
#pragma omp declare reduction(merge: std::vector<Eigen::Matrix3d>: omp_out.insert(omp_out.end(), omp_in.begin(), omp_in.end()))
#endif

#define USE_COTANGENT_WEIGHTS 0 // Otherwise, constant weights will be applied
#define NUM_ITERATIONS 2

class Arap {
public:
    Arap(Eigen::MatrixXd&, Eigen::MatrixXi&);
    ~Arap() = default;

    void updateMovingVertex(int, const Eigen::Vector3f&, Eigen::MatrixXi&, const std::vector<int>&);
    Eigen::MatrixXd computeDeformation(Eigen::MatrixXd&);
private:
    // Neighborhood of vertices (Mapping between vertex id and its neighbor ids)
    std::map<int, std::vector<int>> m_neighborhood;
    void populateNeighborhood(Eigen::MatrixXd&, Eigen::MatrixXi&);

    // Weight matrix used (Constant or Cotangent)
    Eigen::MatrixXd m_weightMatrix;
    void initializeWeightMatrix(Eigen::MatrixXd&, Eigen::MatrixXi&);

    // System matrix
    Eigen::MatrixXd m_systemMatrix;
    void computeSystemMatrix(Eigen::MatrixXd&);

    // ARAP variables (the moving vertex and its position)
    int m_movingVertex{};
    Eigen::Vector3d m_movingVertexPosition{};

    // ARAP variables (the fixed vertices - anchor points)
    std::vector<int> m_fixedVertices;
    void collectFixedVertices(Eigen::MatrixXi&, const std::vector<int>&);
    void updateSystemMatrixOnFixedVertices();

    // Solver
    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;

    // Functions used during deformation
    std::vector<Eigen::Matrix3d> estimateRotations(Eigen::MatrixXd&, Eigen::MatrixXd&);
    Eigen::MatrixXd computeRHS(Eigen::MatrixXd&, std::vector<Eigen::Matrix3d>);
};

#endif // _ARAP_H_

