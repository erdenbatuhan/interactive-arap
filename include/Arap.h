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

#define NUM_ITERATIONS 1

class Arap {
public:
    Arap();
    ~Arap() = default;

    void updateParameters(int);

    std::vector<int> collectFixedVertices(Eigen::MatrixXi&, std::vector<int>&) const;
    Eigen::MatrixXd computeDeformation(Eigen::MatrixXd&, Eigen::MatrixXi&, std::map<int, std::vector<int>>&, std::vector<int>&);
private:
    // The current moving vertex
    int m_movingVertex{};

    // Solver
    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;

    // Functions used during deformation
    static Eigen::MatrixXd computeSystemMatrix(Eigen::MatrixXd&, std::map<int, std::vector<int>>&, Eigen::MatrixXd&);
    static std::vector<Matrix3d> estimateRotations(Eigen::MatrixXd&, Eigen::MatrixXd&, std::map<int, std::vector<int>>&,
                                                   Eigen::MatrixXd&);
    static Eigen::MatrixXd computeRHS(Eigen::MatrixXd&, Eigen::MatrixXd&,
                                      std::map<int, std::vector<int>>&, const std::vector<int>&,
                                      Eigen::MatrixXd, std::vector<Matrix3d>);
    static void updateSystemMatrixOnFixedVertices(Eigen::MatrixXd&, const std::vector<int>&, Eigen::MatrixXd);
};

#endif // _ARAP_H_

