/**
 * Project: Interactive ARAP
 * File:    SimpleArap.cpp
 * Authors: Batuhan Erden, Cansu Yildirim, Anas Shahzad, Alexander Epple
 */

#include "../include/Arap.h"

Arap::Arap(Eigen::MatrixXd& vertices, Eigen::MatrixXi& faces) {
    populateNeighborhood(vertices, faces); // Neighborhood
    initializeWeightMatrix(vertices, faces); // Weights
    computeSystemMatrix(vertices); // LHS
}

void Arap::populateNeighborhood(Eigen::MatrixXd& vertices, Eigen::MatrixXi& faces) {
    m_neighborhood.clear();

#ifdef OMP
    #pragma omp parallel for default(none) \
            shared(m_neighborhood, vertices, faces)
#endif
    for (int i = 0; i < vertices.rows(); i++) { // Iterate over the vertices
        std::vector<int> allNeighbors;
        std::vector<int> distinctNeighbors;

        // Iterate over the edges
        for (int j = 0; j < faces.rows(); j++) { // Iterate over the faces
            if (faces(j, 0) == i) {
                allNeighbors.push_back(faces(j, 1));
                allNeighbors.push_back(faces(j, 2));
            }

            if (faces(j, 1) == i) {
                allNeighbors.push_back(faces(j, 0));
                allNeighbors.push_back(faces(j, 2));
            }

            if (faces(j, 2) == i) {
                allNeighbors.push_back(faces(j, 0));
                allNeighbors.push_back(faces(j, 1));
            }
        }

        for (auto neighbor: allNeighbors) { // Iterate over all the found neighbors of the vertex
            // Push to distinct neighbors if it is not in the list
            if (std::find(distinctNeighbors.begin(), distinctNeighbors.end(), neighbor) == distinctNeighbors.end()) {
                distinctNeighbors.push_back(neighbor);
            }
        }

        #pragma omp critical
        m_neighborhood[i] = distinctNeighbors;
    }
}

void Arap::initializeWeightMatrix(Eigen::MatrixXd& vertices, Eigen::MatrixXi& faces) {
    m_weightMatrix = Eigen::MatrixXd::Ones(vertices.rows(), vertices.rows());

#if USE_COTANGENT_WEIGHTS
    std::vector<Eigen::Vector2i> vertexEdges[vertices.rows()];

#ifdef OMP
    #pragma omp parallel for default(none) \
            shared(vertices, faces, vertexEdges)
#endif
    for (int i = 0; i < vertices.rows(); i++) { // Iterate over the vertices
        std::vector<Eigen::Vector2i> edges;

        for (int j = 0; j < faces.rows(); j++) { // Iterate over the faces
            Eigen::Vector3i face = faces.row(j);

            for (int k = 0; k < 3; k++) { // Iterate over the triangle
                if (face[k] == i) {
                    edges.emplace_back(face[(k + 1) % 3], face[(k + 2) % 3]);
                }
            }
        }

        vertexEdges[i] = edges;
    }

#ifdef OMP
    #pragma omp parallel for default(none) \
            shared(m_neighborhood, m_weightMatrix, vertices, vertexEdges)
#endif
    for (int i = 0; i < vertices.rows(); i++) { // Iterate over the vertices
        for (int neighbor : m_neighborhood[i]) { // Iterate over the neighbors
            double totalAngle = 0.0;

            for (const Eigen::Vector2i& edge : vertexEdges[i]) { // Iterate over the edges
                double norm_bc = (vertices.row(edge[0]) - vertices.row(edge[1])).norm(); // Norm between B and C
                double norm_ac = (vertices.row(i) - vertices.row(edge[1])).norm(); // Norm between A and C
                double norm_ab = (vertices.row(i) - vertices.row(edge[0])).norm(); // Norm between A and B

                // From cosine law
                double beta = acos(((norm_ab * norm_ab) + (norm_bc * norm_bc) - (norm_ac * norm_ac)) / (2 * norm_ab * norm_bc));

                // Add to total angle if one of the points on the edge is the current neighbor
                totalAngle += (edge[0] == neighbor) * abs(tan(M_PI_2 - beta));
                totalAngle += (edge[1] == neighbor) * abs(tan(M_PI_2 - beta));
            }

            m_weightMatrix(i, neighbor) = abs(totalAngle) / 2;
        }

        m_weightMatrix(i, i) = 1.0; // Override the diagonal entry
    }
#endif
}

void Arap::computeSystemMatrix(Eigen::MatrixXd& vertices) {
    m_systemMatrix = Eigen::MatrixXd::Zero(vertices.rows(), vertices.rows());

#ifdef OMP
    #pragma omp parallel for default(none) \
            shared(m_neighborhood, m_weightMatrix, m_systemMatrix, vertices)
#endif
    for (int i = 0; i < vertices.rows(); i++) { // Iterate over the vertices
        for (int neighbor : m_neighborhood[i]) { // Iterate over the neighbors
            m_systemMatrix(i, i) += m_weightMatrix(i, neighbor);
            m_systemMatrix(i, neighbor) -= m_weightMatrix(i, neighbor);
        }
    }
}

void Arap::collectFixedVertices(Eigen::MatrixXi& faces, const std::vector<int>& anchorFaces) {
    m_fixedVertices.clear();
    m_fixedVertices.reserve(anchorFaces.size() * faces.row(0).cols() + 1);

    // Add the vertices of each face to the fixed vertices
    for (int anchorFace : anchorFaces) {
        Eigen::VectorXi faceVertices = faces.row(anchorFace);

        for (int j = 0; j < faces.cols(); j++) {
            m_fixedVertices.push_back(faceVertices(j));
        }
    }

    // Add the selected vertex to the fixed vertices
    m_fixedVertices.push_back(m_movingVertex);
}

void Arap::updateSystemMatrixOnFixedVertices() {
    // Update system matrix on fixed vertices to keep the fixed vertices stay where they are
    for (int fixedVertex : m_fixedVertices) { // Iterate over the fixed vertices
        m_systemMatrix.row(fixedVertex).setZero();
        m_systemMatrix(fixedVertex, fixedVertex) = 1;
    }
}

void Arap::updateMovingVertex(const int movingVertex, const Eigen::Vector3f& movingVertexPosition,
                              Eigen::MatrixXi& faces, const std::vector<int>& anchorFaces) {
    m_movingVertex = movingVertex;
    m_movingVertexPosition = movingVertexPosition.cast<double>();

    collectFixedVertices(faces, anchorFaces);
    updateSystemMatrixOnFixedVertices();
}

std::vector<Eigen::Matrix3d> Arap::estimateRotations(Eigen::MatrixXd& deformedVertices, Eigen::MatrixXd& vertices) {
    std::vector<Eigen::Matrix3d> rotationMatrices;
    rotationMatrices.reserve(vertices.rows());

#ifdef OMP
    #pragma omp parallel for default(none) \
            shared(m_neighborhood, m_weightMatrix, vertices, deformedVertices) \
            reduction(merge: rotationMatrices)
#endif
    for (int i = 0; i < vertices.rows(); i++) { // Iterate over the vertices
        const long numNeighbors = (long) (m_neighborhood[i].size());

        // The definitions for the matrices P, D and P_prime can be found in the paper!
        Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, numNeighbors);
        Eigen::MatrixXd D = Eigen::MatrixXd::Zero(numNeighbors, numNeighbors);
        Eigen::MatrixXd P_prime = Eigen::MatrixXd::Zero(3, numNeighbors);

        for (int j = 0; j < numNeighbors; j++) { // Iterate over the neighbors
            P.col(j) = vertices.row(i) - vertices.row(m_neighborhood[i][j]);
            D(j, j) = m_weightMatrix(i, m_neighborhood[i][j]);
            P_prime.col(j) = deformedVertices.row(i) - deformedVertices.row(m_neighborhood[i][j]);
        }

        // S, the covariance matrix
        Eigen::Matrix3d S = P * D * P_prime.transpose();

        // SVD
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(S, Eigen::ComputeThinU | Eigen::ComputeThinV);
        const Eigen::Matrix3d& U = svd.matrixU();
        const Eigen::Matrix3d& V = svd.matrixV();

        // Computation of matrix I is necessary since UV' is only orthogonal, but not necessarily a rotation matrix
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        I(2, 2) = (U * V.transpose()).determinant();

        // Add the rotation matrix R to the list
        Eigen::Matrix3d R = V * I * U.transpose(); // or (U * I * V.T).T
        rotationMatrices.push_back(R);
    }

    return rotationMatrices;
}

Eigen::MatrixXd Arap::computeRHS(Eigen::MatrixXd& vertices, std::vector<Eigen::Matrix3d> rotationMatrices) {
    Eigen::MatrixXd rhs = Eigen::MatrixXd::Zero(vertices.rows(), 3);

#ifdef OMP
    #pragma omp parallel for default(none) \
            shared(m_neighborhood, m_weightMatrix, m_fixedVertices, vertices, rotationMatrices, rhs)
#endif
    for (int i = 0; i < vertices.rows(); i++) { // Iterate over the vertices
        Eigen::Vector3d rhsRow = Eigen::Vector3d(0.0, 0.0, 0.0);

        if (std::find(m_fixedVertices.begin(), m_fixedVertices.end(), i) != m_fixedVertices.end()) { // Current vertex is a fixed vertex
            rhsRow = (i != m_movingVertex) ? vertices.row(i) : m_movingVertexPosition; // If the vertex is the moving one, get the new position
        } else { // Current vertex is not a fixed vertex but a to-be-deformed vertex
            for (int neighbor : m_neighborhood[i]) { // Iterate over the neighbors
                rhsRow += 0.5 * m_weightMatrix(i, neighbor) *
                          (vertices.row(i) - vertices.row(neighbor)) *
                          (rotationMatrices[i] + rotationMatrices[neighbor]);
            }
        }

        rhs.row(i) = rhsRow;
    }

    return rhs;
}

Eigen::MatrixXd Arap::computeDeformation(Eigen::MatrixXd& vertices) {
    Eigen::MatrixXd deformedVertices = safeReplicate(vertices); // Deformed vertices are stored in a different matrix

    // Optimize over some iterations
    for (int i = 0; i < NUM_ITERATIONS; i++) {
        // Estimate rotations
        std::vector<Eigen::Matrix3d> rotationMatrices = estimateRotations(deformedVertices, vertices);

        // Compute RHS
        Eigen::MatrixXd rhs = computeRHS(vertices, rotationMatrices);

        // Solve the system
        solver.compute(m_systemMatrix.sparseView());
        deformedVertices = solver.solve(rhs);
    }

    return deformedVertices;
}

