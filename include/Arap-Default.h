/**
 * Project: Interactive ARAP
 * File:    Arap-Default.h
 * Authors: Batuhan Erden, Cansu Yildirim, Anas Shahzad, Alexander Epple
 */

#ifndef _ARAP_DEFAULT_H_
#define _ARAP_DEFAULT_H_

#include "Arap.h"

#ifdef OMP
#include <omp.h>
#endif

#define NUM_ITERATIONS 2

class ArapDefault : public Arap
{
public:
    ArapDefault() = default;
    ~ArapDefault() = default;

    virtual void updateMovingVertex(int, const Eigen::Vector3f&) override;

    virtual std::vector<int> collectFixedVertices(Eigen::MatrixXi&, const std::vector<int>&) const override;
    virtual Eigen::MatrixXd computeDeformation(Eigen::MatrixXd&, Eigen::MatrixXi&,
        std::map<int, std::vector<int>>&, const std::vector<int>&) override;
private:
    // Solver
    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;

    // Functions used during deformation
    static Eigen::MatrixXd initializeWeightMatrix(Eigen::MatrixXd&, std::map<int, std::vector<int>>&);
    static Eigen::MatrixXd computeSystemMatrix(Eigen::MatrixXd&, std::map<int, std::vector<int>>&,
        const std::vector<int>&, Eigen::MatrixXd&);
    static void estimateRotations(Eigen::MatrixXd&, Eigen::MatrixXd&, std::map<int, std::vector<int>>&,
        Eigen::MatrixXd&, Eigen::Matrix3d*);
    Eigen::MatrixXd computeRHS(Eigen::MatrixXd&, std::map<int, std::vector<int>>&, const std::vector<int>&,
        Eigen::MatrixXd, Eigen::Matrix3d*) const;
};

#endif // _ARAP_DEFAULT_H_
