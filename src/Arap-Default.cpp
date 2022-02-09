/**
 * Project: Interactive ARAP
 * File:    Arap-Default.cpp
 * Authors: Batuhan Erden, Cansu Yildirim, Anas Shahzad, Alexander Epple
 */

#include "../include/Arap-Default.h"

void ArapDefault::updateMovingVertex(
  const int movingVertex,
  const Eigen::Vector3f& movingVertexPosition
)
{
    m_movingVertex = movingVertex;
    m_movingVertexPosition = movingVertexPosition.cast<double>();
}

std::vector<int> ArapDefault::collectFixedVertices(
  Eigen::MatrixXi& faces,
  const std::vector<int>& anchorFaces
) const
{
    std::vector<int> fixedVertices;

    // Add the vertices of each face to the fixed vertices
    for (int anchorFace : anchorFaces) {
        Eigen::VectorXi faceVertices = faces.row(anchorFace);

        for (int j = 0; j < faces.cols(); j++) {
            fixedVertices.push_back(faceVertices(j));
        }
    }

    // Add the selected vertex to the fixed vertices
    fixedVertices.push_back(m_movingVertex);

    return fixedVertices;
}

Eigen::MatrixXd ArapDefault::initializeWeightMatrix(
  Eigen::MatrixXd& vertices,
  std::map<int, std::vector<int>>& neighborhood
)
{
    Eigen::MatrixXd weightMatrix = Eigen::MatrixXd::Ones(vertices.rows(), vertices.rows());
    return weightMatrix;
}

Eigen::MatrixXd ArapDefault::computeSystemMatrix(
  Eigen::MatrixXd& vertices, std::map<int,
  std::vector<int>>& neighborhood,
  const std::vector<int>& fixedVertices,
  Eigen::MatrixXd& weightMatrix
)
{
    Eigen::MatrixXd systemMatrix = Eigen::MatrixXd::Zero(vertices.rows(), vertices.rows());

#ifdef OMP
    #pragma omp parallel for default(none) \
            shared(vertices, neighborhood, weightMatrix, systemMatrix)
#endif
    for (int i = 0; i < vertices.rows(); i++) { // Iterate over the vertices
        for (int neighbor : neighborhood[i]) { // Iterate over the neighbors
            systemMatrix(i, i) += weightMatrix(i, neighbor);
            systemMatrix(i, neighbor) -= weightMatrix(i, neighbor);
        }
    }

    // Update system matrix on fixed vertices to keep the fixed vertices stay where they are
    for (int fixedVertex : fixedVertices) { // Iterate over the fixed vertices
        systemMatrix.row(fixedVertex).setZero();
        systemMatrix(fixedVertex, fixedVertex) = 1;
    }

    return systemMatrix;
}

void ArapDefault::estimateRotations(
  Eigen::MatrixXd& deformedVertices,
  Eigen::MatrixXd& vertices,
  std::map<int, std::vector<int>>& neighborhood,
  Eigen::MatrixXd& weightMatrix,
  Eigen::Matrix3d* rotationMatrices
)
{
#ifdef OMP
    #pragma omp parallel for default(none) \
            shared(vertices, neighborhood, weightMatrix, deformedVertices, rotationMatrices)
#endif
    for (int i = 0; i < vertices.rows(); i++) { // Iterate over the vertices
        const long numNeighbors = (long) (neighborhood[i].size());

        // The definitions for the matrices P, D and P_prime can be found in the paper!
        Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, numNeighbors);
        Eigen::MatrixXd D = Eigen::MatrixXd::Zero(numNeighbors, numNeighbors);
        Eigen::MatrixXd P_prime = Eigen::MatrixXd::Zero(3, numNeighbors);

        for (int j = 0; j < numNeighbors; j++) { // Iterate over the neighbors
            P.col(j) = vertices.row(i) - vertices.row(neighborhood[i][j]);
            D(j, j) = weightMatrix(i, neighborhood[i][j]);
            P_prime.col(j) = deformedVertices.row(i) - deformedVertices.row(neighborhood[i][j]);
        }

        // S, the covariance matrix
        Eigen::Matrix3d S = P * D * P_prime.transpose();

        // SVD
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(S, Eigen::ComputeThinU | Eigen::ComputeThinV);
        const Eigen::Matrix3d& U = svd.matrixU();
        const Eigen::Matrix3d& V = svd.matrixV();

        // Computation of matrix I is necessary since UV' is only orthogonal, but not necessarily a rotation matrix
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        I(2, 2) = (U * V.transpose()).determinant();

        // Add the rotation matrix R to the list
        Eigen::Matrix3d R = V * I * U.transpose(); // or (U * I * V.T).T
        rotationMatrices[i] = R;
    }
}

Eigen::MatrixXd ArapDefault::computeRHS(
  Eigen::MatrixXd& vertices,
  std::map<int, std::vector<int>>& neighborhood,
  const std::vector<int>& fixedVertices,
  Eigen::MatrixXd weightMatrix,
  Eigen::Matrix3d* rotationMatrices
) const
{
    Eigen::MatrixXd rhs = Eigen::MatrixXd::Zero(vertices.rows(), 3);

#ifdef OMP
    #pragma omp parallel for default(none) \
            shared(vertices, neighborhood, fixedVertices, weightMatrix, rotationMatrices, rhs)
#endif
    for (int i = 0; i < vertices.rows(); i++) { // Iterate over the vertices
        Eigen::Vector3d rhsRow = Eigen::Vector3d(0.0, 0.0, 0.0);

        if (std::find(fixedVertices.begin(), fixedVertices.end(), i) != fixedVertices.end()) { // Current vertex is a fixed vertex
            rhsRow = (i != m_movingVertex) ? vertices.row(i) : m_movingVertexPosition; // If the vertex is the moving one, get the new position
        } else { // Current vertex is not a fixed vertex but a to-be-deformed vertex
            const long numNeighbors = (long) (neighborhood[i].size());

            for (int j = 0; j < numNeighbors; j++) { // Iterate over the neighbors
                rhsRow += 0.5 * weightMatrix(i, j) *
                          (vertices.row(i) - vertices.row(neighborhood[i][j])) *
                          (rotationMatrices[i] + rotationMatrices[neighborhood[i][j]]);
            }
        }

        rhs.row(i) = rhsRow;
    }

    return rhs;
}

Eigen::MatrixXd ArapDefault::computeDeformation(
  Eigen::MatrixXd& vertices,
  Eigen::MatrixXi& faces,
  std::map<int, std::vector<int>>& neighborhood,
  const std::vector<int>& anchorFaceIds
)
{
    Eigen::MatrixXd deformedVertices = safeReplicate(vertices); // Deformed vertices are stored in a different matrix

    std::vector<int> fixedVertices = collectFixedVertices(faces, anchorFaceIds); // Collect all fixed vertices in a list

    Eigen::MatrixXd weightMatrix = initializeWeightMatrix(vertices, neighborhood); // Weights
    Eigen::MatrixXd systemMatrix = computeSystemMatrix(vertices, neighborhood, fixedVertices, weightMatrix); // LHS

    // Optimize over some iterations
    for (int i = 0; i < NUM_ITERATIONS; i++) {
        // Estimate rotations
        std::vector<Eigen::Matrix3d> rotationMatrices(vertices.rows());
        estimateRotations(deformedVertices, vertices, neighborhood, weightMatrix, rotationMatrices.data());

        // Compute RHS
        Eigen::MatrixXd rhs = computeRHS(vertices, neighborhood, fixedVertices, weightMatrix, rotationMatrices.data());

        // Solve the system
        solver.compute(systemMatrix.sparseView());
        deformedVertices = solver.solve(rhs);
    }

    return deformedVertices;
}
