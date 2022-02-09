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
#endif

#define USE_COTANGENT_WEIGHTS 1 // Otherwise, constant weights will be applied
#define NUM_ITERATIONS 2

class Arap {
public:
    Arap();
    ~Arap() = default;

    void updateMovingVertex(int, const Eigen::Vector3f&);

    std::vector<int> collectFixedVertices(Eigen::MatrixXi&, const std::vector<int>&) const;
    Eigen::MatrixXd computeDeformation(Eigen::MatrixXd&, Eigen::MatrixXi&,
                                       std::map<int, std::vector<int>>&, const std::vector<int>&);
private:
    // The current moving vertex
    int m_movingVertex{};
    Eigen::Vector3d m_movingVertexPosition{};

    // Solver
    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;

    // Functions used during deformation
    static Eigen::MatrixXd initializeWeightMatrix(Eigen::MatrixXd&, Eigen::MatrixXi&, std::map<int, std::vector<int>>&);
    static Eigen::MatrixXd computeSystemMatrix(Eigen::MatrixXd&, std::map<int, std::vector<int>>&,
                                               const std::vector<int>&, Eigen::MatrixXd&);
    static void estimateRotations(Eigen::MatrixXd&, Eigen::MatrixXd&, std::map<int, std::vector<int>>&,
                                  Eigen::MatrixXd&, Eigen::Matrix3d*);
    Eigen::MatrixXd computeRHS(Eigen::MatrixXd&, std::map<int, std::vector<int>>&, const std::vector<int>&,
                               Eigen::MatrixXd, Eigen::Matrix3d*) const;
};

#endif // _ARAP_H_

