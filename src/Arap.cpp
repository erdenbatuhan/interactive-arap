/**
 * Project: Interactive ARAP
 * File:    SimpleArap.cpp
 * Authors: Batuhan Erden, Cansu Yildirim, Anas Shahzad, Alexander Epple
 */

#include "../include/Arap.h"

Arap::Arap(Eigen::MatrixXd& vertices, Eigen::MatrixXi& faces) {
    // Copy the vertices to a new matrix before any deformation operation
    m_undeformedVertices = safeReplicate(vertices);

    populateNeighborhood(faces); // Neighborhood
    initializeWeightMatrix(faces); // Weights
    computeSystemMatrix(); // LHS
}

void Arap::populateNeighborhood(Eigen::MatrixXi& faces) {
    m_neighborhood.clear();

#ifdef OMP
    #pragma omp parallel for default(none) \
            shared(m_undeformedVertices, m_neighborhood, faces)
#endif
    for (int i = 0; i < m_undeformedVertices.rows(); i++) { // Iterate over the vertices
        std::vector<int> allNeighbors;
        std::vector<int> distinctNeighbors;

        // Iterate over the edges
        for (int j = 0; j < faces.rows(); j++) { // Iterate over the faces
            for (int k = 0; k < 3; k++) { // Iterate over the points
                if (faces(j, k) == i) {
                    allNeighbors.push_back(faces(j, (k + 1) % 3));
                    allNeighbors.push_back(faces(j, (k + 2) % 3));
                }
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

void Arap::initializeWeightMatrix(Eigen::MatrixXi& faces) {
#if not USE_COTANGENT_WEIGHTS // Constant weights
    m_weightMatrix = Eigen::MatrixXd::Ones(m_undeformedVertices.rows(), m_undeformedVertices.rows());
#else // Cotangent weights
    m_weightMatrix = Eigen::MatrixXd::Zero(m_undeformedVertices.rows(), m_undeformedVertices.rows());
    std::vector<Eigen::Vector2i> vertexEdges[m_undeformedVertices.rows()];

#ifdef OMP
    #pragma omp parallel for default(none) \
            shared(m_undeformedVertices, faces, vertexEdges)
#endif
    for (int i = 0; i < m_undeformedVertices.rows(); i++) { // Iterate over the vertices
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
            shared(m_undeformedVertices, m_neighborhood, m_weightMatrix, vertexEdges)
#endif
    for (int i = 0; i < m_undeformedVertices.rows(); i++) { // Iterate over the vertices
        for (int neighbor : m_neighborhood[i]) { // Iterate over the neighbors
            double totalAngle = 0.0;

            for (const Eigen::Vector2i& edge : vertexEdges[i]) { // Iterate over the edges
                double norm_bc = (m_undeformedVertices.row(edge[0]) - m_undeformedVertices.row(edge[1])).norm(); // Norm between B and C
                double norm_ac = (m_undeformedVertices.row(i) - m_undeformedVertices.row(edge[1])).norm(); // Norm between A and C
                double norm_ab = (m_undeformedVertices.row(i) - m_undeformedVertices.row(edge[0])).norm(); // Norm between A and B

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

void Arap::computeSystemMatrix() {
    m_systemMatrix = Eigen::MatrixXd::Zero(m_undeformedVertices.rows(), m_undeformedVertices.rows());

#ifdef OMP
    #pragma omp parallel for default(none) \
            shared(m_undeformedVertices, m_neighborhood, m_weightMatrix, m_systemMatrix)
#endif
    for (int i = 0; i < m_undeformedVertices.rows(); i++) { // Iterate over the vertices
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

std::vector<Eigen::Matrix3d> Arap::estimateRotations(Eigen::MatrixXd& deformedVertices) {
    std::vector<Eigen::Matrix3d> rotationMatrices;
    rotationMatrices.reserve(m_undeformedVertices.rows());

#ifdef OMP
    #pragma omp parallel for default(none) \
            shared(m_undeformedVertices, m_neighborhood, m_weightMatrix, deformedVertices) \
            reduction(merge: rotationMatrices)
#endif
    for (int i = 0; i < m_undeformedVertices.rows(); i++) { // Iterate over the vertices
        const long numNeighbors = (long) (m_neighborhood[i].size());

        // The definitions for the matrices P, D and P_prime can be found in the paper!
        Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, numNeighbors);
        Eigen::MatrixXd D = Eigen::MatrixXd::Zero(numNeighbors, numNeighbors);
        Eigen::MatrixXd P_prime = Eigen::MatrixXd::Zero(3, numNeighbors);

        for (int j = 0; j < numNeighbors; j++) { // Iterate over the neighbors
            P.col(j) = m_undeformedVertices.row(i) - m_undeformedVertices.row(m_neighborhood[i][j]);
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

Eigen::MatrixXd Arap::computeRHS(std::vector<Eigen::Matrix3d> rotationMatrices) {
    Eigen::MatrixXd rhs = Eigen::MatrixXd::Zero(m_undeformedVertices.rows(), 3);

#ifdef OMP
    #pragma omp parallel for default(none) \
            shared(m_undeformedVertices, m_neighborhood, m_weightMatrix, m_fixedVertices, rotationMatrices, rhs)
#endif
    for (int i = 0; i < m_undeformedVertices.rows(); i++) { // Iterate over the vertices
        Eigen::Vector3d rhsRow = Eigen::Vector3d(0.0, 0.0, 0.0);

        if (std::find(m_fixedVertices.begin(), m_fixedVertices.end(), i) != m_fixedVertices.end()) { // Current vertex is a fixed vertex
            rhsRow = (i != m_movingVertex) ? m_undeformedVertices.row(i) : m_movingVertexPosition; // If the vertex is the moving one, get the new position
        } else { // Current vertex is not a fixed vertex but a to-be-deformed vertex
            for (int neighbor : m_neighborhood[i]) { // Iterate over the neighbors
                rhsRow += 0.5 * m_weightMatrix(i, neighbor) *
                          (m_undeformedVertices.row(i) - m_undeformedVertices.row(neighbor)) *
                          (rotationMatrices[i] + rotationMatrices[neighbor]);
            }
        }

        rhs.row(i) = rhsRow;
    }

    return rhs;
}

double Arap::computeRigidityEnergy(Eigen::MatrixXd& deformedVertices, std::vector<Eigen::Matrix3d> rotationMatrices) {
    double rigidityEnergy = 0.0;  // rigidity energy

    for (int i = 0; i < m_undeformedVertices.rows(); i++) { // Iterate over the undeformed vertices
        double rigidityEnergyPerCell = 0.0;  // energy per cell

        for (int neighbor : m_neighborhood[i]) { // Iterate over the neighbors
            Eigen::Vector3d deformedPositionsDiff = deformedVertices.row(i) - deformedVertices.row(neighbor);
            Eigen::Vector3d undeformedPositionsDiff  = m_undeformedVertices.row(i) - m_undeformedVertices.row(neighbor);

            rigidityEnergyPerCell += m_weightMatrix(i, neighbor) * (deformedPositionsDiff - rotationMatrices[i] * undeformedPositionsDiff).squaredNorm();
        }

        rigidityEnergy += m_weightMatrix(i,i) * rigidityEnergyPerCell;
    }

    return rigidityEnergy;
}

Eigen::MatrixXd Arap::computeDeformation(Eigen::MatrixXd& currentVertices) {
    // Vertices before this deformation are copied into a new matrix, deformed vertices, which will be updated with deformation
    Eigen::MatrixXd deformedVertices = safeReplicate(currentVertices);

    // Start the timer
    const std::chrono::time_point<std::chrono::system_clock> t0 = std::chrono::system_clock::now();

    // Optimize over some iterations
    auto previousRigidityEnergy = DBL_MAX;
    for (int i = 0; i < NUM_ITERATIONS; i++) {
        // Estimate rotations
        std::vector<Eigen::Matrix3d> rotationMatrices = estimateRotations(deformedVertices);

        // Compute RHS
        Eigen::MatrixXd rhs = computeRHS(rotationMatrices);

        // Solve the system
        solver.compute(m_systemMatrix.sparseView());
        deformedVertices = solver.solve(rhs);

        // Performance analysis for each iteration
        const double rigidityEnergy = computeRigidityEnergy(deformedVertices, rotationMatrices);
        printf("Iteration %d: rigidity energy = %e\n", i, rigidityEnergy);

        // Stop early if the solution is good enough
        if (abs(previousRigidityEnergy - rigidityEnergy) < LOWER_ENERGY_THRESHOLD) {
            i = NUM_ITERATIONS;
            printf("Iteration %d: Energy threshold %f reached! Stopping early..\n", i, LOWER_ENERGY_THRESHOLD);
        } else {
            previousRigidityEnergy = rigidityEnergy;
        }
    }

    // End the timer and print the duration
    const std::chrono::time_point<std::chrono::system_clock> t1 = std::chrono::system_clock::now();
    printf("Took %f seconds to deform..\n\n", std::chrono::duration<double>(t1 - t0).count());

    return deformedVertices;
}

