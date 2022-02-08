/**
 * Project: Interactive ARAP
 * File:    SimpleArap.cpp
 * Authors: Batuhan Erden, Cansu Yildirim, Anas Shahzad, Alexander Epple
 */

#include "../include/Arap.h"

Arap::Arap() = default;

void Arap::updateParameters(const int movingVertex) {
    m_movingVertex = movingVertex;
}

std::vector<int> Arap::collectFixedVertices(Eigen::MatrixXi& faces, std::vector<int>& anchorFaces) const {
    std::vector<int> fixedVertices;

    // Add the vertices of each face to the fixed vertices
    for (int anchorFace : anchorFaces) {
        Eigen::VectorXi faceVertices = faces.row(anchorFace);

        for (int j = 0; j < faces.cols(); j++) {
            fixedVertices.push_back(faces.row(anchorFace)(j));
        }
    }

    // Add the selected vertex to the fixed vertices
    fixedVertices.push_back(m_movingVertex);

    return fixedVertices;
}

Eigen::MatrixXd initializeWeightMatrix(Eigen::MatrixXd& vertices, std::map<int, std::vector<int>>& neighborhood) {
    Eigen::MatrixXd weightMatrix = Eigen::MatrixXd::Ones(vertices.rows(), vertices.rows());
    return weightMatrix;
}

Eigen::MatrixXd Arap::computeSystemMatrix(Eigen::MatrixXd& vertices, std::map<int, std::vector<int>>& neighborhood,
                                          Eigen::MatrixXd& weightMatrix) {
    Eigen::MatrixXd systemMatrix = MatrixXd::Zero(vertices.rows(), vertices.rows());

    for (int i = 0; i < vertices.rows(); i++) { // Iterate over the vertices
        for (int neighbor : neighborhood[i]) { // Iterate over the neighbors
            systemMatrix(i, i) += weightMatrix(i, neighbor);
            systemMatrix(i, neighbor) -= weightMatrix(i, neighbor);
        }
    }

    return systemMatrix;
}

std::vector<Matrix3d> Arap::estimateRotations(Eigen::MatrixXd& deformedVertices, Eigen::MatrixXd& vertices,
                                              std::map<int, std::vector<int>>& neighborhood,
                                              Eigen::MatrixXd& weightMatrix) {
    std::vector<Matrix3d> rotationMatrices;

    for (int i = 0; i < vertices.rows(); i++) { // Iterate over the vertices
        const long numNeighbors = (long) (neighborhood[i].size());

        // The definitions for the matrices P, D and P_prime can be found in the paper!
        MatrixXd P = MatrixXd::Zero(3, numNeighbors);
        MatrixXd D = MatrixXd::Zero(numNeighbors, numNeighbors);
        MatrixXd P_prime = MatrixXd::Zero(3, numNeighbors);

        for (int j = 0; j < numNeighbors; j++) { // Iterate over the neighbors
            P.col(j) = vertices.row(i) - vertices.row(neighborhood[i][j]);
            D(j, j) = weightMatrix(i, neighborhood[i][j]);
            P_prime.col(j) = deformedVertices.row(i) - deformedVertices.row(neighborhood[i][j]);
        }

        // S, the covariance matrix
        Matrix3d S = P * D * P_prime.transpose();

        // SVD
        JacobiSVD<MatrixXd> svd(S, ComputeThinU | ComputeThinV);
        const Matrix3d& U = svd.matrixU();
        const Matrix3d& V = svd.matrixV();

        // Computation of matrix I is necessary since UV' is only orthogonal, but not necessarily a rotation matrix
        Matrix3d I = Matrix3d::Identity();
        I(2, 2) = (U * V.transpose()).determinant();

        // Add the rotation matrix R to the list
        Matrix3d R = V * I * U.transpose(); // or (U * I * V.T).T
        rotationMatrices.push_back(R);
    }

    return rotationMatrices;
}

Eigen::MatrixXd Arap::computeRHS(Eigen::MatrixXd& deformedVertices, Eigen::MatrixXd& vertices,
                                 std::map<int, std::vector<int>>& neighborhood, const std::vector<int>& fixedVertices,
                                 Eigen::MatrixXd weightMatrix, std::vector<Matrix3d> rotationMatrices) {
    Eigen::MatrixXd rhs = MatrixXd::Zero(vertices.rows(), 3);

    for (int i = 0; i < vertices.rows(); i++) { // Iterate over the vertices
        Eigen::Vector3d rhsRow = Eigen::Vector3d(0.0, 0.0, 0.0);

        if (std::find(fixedVertices.begin(), fixedVertices.end(), i) != fixedVertices.end()) { // Current vertex is a fixed vertex
            rhsRow = vertices.row(i);
        } else { // Current vertex is not a fixed vertex but a to-be-deformed vertex
            for (int neighbor : neighborhood[i]) { // Iterate over the neighbors
                rhsRow += 0.5 * weightMatrix(i, neighbor) *
                          (vertices.row(i) - vertices.row(neighbor)) *
                          (rotationMatrices[i] + rotationMatrices[neighbor]);
            }
        }

        rhs.row(i) = rhsRow;
    }

    return rhs;
}

void Arap::updateSystemMatrixOnFixedVertices(Eigen::MatrixXd& vertices, const std::vector<int>& fixedVertices,
                                             Eigen::MatrixXd systemMatrix) {
    for (auto it = fixedVertices.begin(); it < fixedVertices.end(); it++) { // Iterate over the vertices
        systemMatrix.row(*it).setZero();
        systemMatrix(*it, *it) = 1;
    }
}

Eigen::MatrixXd Arap::computeDeformation(Eigen::MatrixXd& vertices, Eigen::MatrixXi& faces,
                                         std::map<int, std::vector<int>>& neighborhood, std::vector<int>& anchorFaceIds) {
    Eigen::MatrixXd deformedVertices; // Deformed vertices are stored in a different matrix
    std::vector<int> fixedVertices = collectFixedVertices(faces, anchorFaceIds); // Collect all fixed vertices in a list

    Eigen::MatrixXd weightMatrix = initializeWeightMatrix(vertices, neighborhood); // Weights
    Eigen::MatrixXd systemMatrix = computeSystemMatrix(vertices, neighborhood, weightMatrix); // LHS

    // Optimize over some iterations
    for (int i = 0; i < NUM_ITERATIONS; i++) {
        // Estimate rotations
        const std::vector<Matrix3d> rotationMatrices = estimateRotations(deformedVertices, vertices, neighborhood, weightMatrix);

        // Compute RHS
        Eigen::MatrixXd rhs = computeRHS(deformedVertices, vertices, neighborhood, fixedVertices, weightMatrix, rotationMatrices);

        // Update system matrix on fixed vertices to keep the fixed vertices stay where they are
        updateSystemMatrixOnFixedVertices(vertices, fixedVertices, systemMatrix);

        // Solve the system
        solver.compute(systemMatrix.sparseView());
        deformedVertices = solver.solve(rhs);
    }

    return deformedVertices;
}

