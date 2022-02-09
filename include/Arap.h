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

    void updateMovingVertex(int);
    void updateMovingVertexPosition(const Eigen::Vector3f&);

    std::vector<int> collectFixedVertices(Eigen::MatrixXi&, const std::vector<int>&) const;
    Eigen::MatrixXd computeDeformation(Eigen::MatrixXd&, Eigen::MatrixXi&,
                                       std::map<int, std::vector<int>>&, const std::vector<int>&);

    int getMovingVertex() const;
private:
    // The current moving vertex
    int m_movingVertex{};
    Eigen::Vector3d m_movingVertexPosition{};

    // Solver
    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;

    // Functions used during deformation
    static Eigen::MatrixXd initializeWeightMatrix(Eigen::MatrixXd&, std::map<int, std::vector<int>>&);
    static Eigen::MatrixXd computeSystemMatrix(Eigen::MatrixXd&, std::map<int, std::vector<int>>&,
                                               const std::vector<int>&, Eigen::MatrixXd&);
    static std::vector<Eigen::Matrix3d> estimateRotations(Eigen::MatrixXd&, Eigen::MatrixXd&,
                                                          std::map<int, std::vector<int>>&, Eigen::MatrixXd&);
    Eigen::MatrixXd computeRHS(Eigen::MatrixXd&, std::map<int, std::vector<int>>&, const std::vector<int>&,
                               Eigen::MatrixXd, std::vector<Eigen::Matrix3d>) const;
};

#endif // _ARAP_H_

