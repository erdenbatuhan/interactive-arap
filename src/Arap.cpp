/**
 * Project: Interactive ARAP
 * File:    SimpleArap.cpp
 * Authors: Batuhan Erden, Cansu Yildirim, Anas Shahzad, Alexander Epple
 */

#include "../include/Arap.h"

Arap::Arap() {
#ifdef OMP
    // Init Eigen in parallel mode
    Eigen::initParallel();

    // Determine the number of threads
    int numThreads;
    #pragma omp parallel default(none) shared(numThreads)
    numThreads = omp_get_thread_num();

    // Set number of threads Eigen can use (usually it is better to use half since Eigen uses almost 100% of CPU capacity)
    Eigen::setNbThreads(numThreads / 2);
#endif
}

void Arap::populateNeighborhood(Eigen::MatrixXi& faces) {
    m_neighborhood.clear();

    auto& undeformedVertices = m_undeformedVertices;
    auto& neighborhood = m_neighborhood;

#ifdef OMP
    #pragma omp parallel for default(none) \
            shared(undeformedVertices, neighborhood, faces)
#endif
    for (int i = 0; i < undeformedVertices.rows(); i++) { // Iterate over the vertices
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
        neighborhood[i] = distinctNeighbors;
    }
}

void Arap::initializeWeightMatrix(Eigen::MatrixXi& faces) {
#if not COTANGENT_WEIGHTING // Constant weights
    m_weightMatrix = Eigen::MatrixXd::Ones(m_undeformedVertices.rows(), m_undeformedVertices.rows());
#else // Cotangent weights
    m_weightMatrix = Eigen::MatrixXd::Zero(m_undeformedVertices.rows(), m_undeformedVertices.rows());
    std::vector<std::vector<Eigen::Vector2i>> vertexEdges(m_undeformedVertices.rows());

    auto& undeformedVertices = m_undeformedVertices;
    auto& neighborhood = m_neighborhood;
    auto& weightMatrix = m_weightMatrix;

#ifdef OMP
    #pragma omp parallel for default(none) \
            shared(undeformedVertices, faces, vertexEdges)
#endif
    for (int i = 0; i < undeformedVertices.rows(); i++) { // Iterate over the vertices
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
            shared(undeformedVertices, neighborhood, weightMatrix, vertexEdges)
#endif
    for (int i = 0; i < undeformedVertices.rows(); i++) { // Iterate over the vertices
        for (int neighbor : neighborhood[i]) { // Iterate over the neighbors
            double totalAngle = 0.0;

            for (const Eigen::Vector2i& edge : vertexEdges[i]) { // Iterate over the edges
                double norm_bc = (undeformedVertices.row(edge[0]) - undeformedVertices.row(edge[1])).norm(); // Norm between B and C
                double norm_ac = (undeformedVertices.row(i) - undeformedVertices.row(edge[1])).norm(); // Norm between A and C
                double norm_ab = (undeformedVertices.row(i) - undeformedVertices.row(edge[0])).norm(); // Norm between A and B

                // From cosine law
                double beta = acos(((norm_ab * norm_ab) + (norm_bc * norm_bc) - (norm_ac * norm_ac)) / (2 * norm_ab * norm_bc));

                // Add to total angle if one of the points on the edge is the current neighbor
                totalAngle += (edge[0] == neighbor) * abs(tan(M_PI_2 - beta));
                totalAngle += (edge[1] == neighbor) * abs(tan(M_PI_2 - beta));
            }

            weightMatrix(i, neighbor) = abs(totalAngle) / 2;
        }

        weightMatrix(i, i) = 1.0; // Override the diagonal entry
    }
#endif
}

void Arap::computeSystemMatrix() {
    m_systemMatrix = Eigen::MatrixXd::Zero(m_undeformedVertices.rows(), m_undeformedVertices.rows());

    auto& undeformedVertices = m_undeformedVertices;
    auto& neighborhood = m_neighborhood;
    auto& weightMatrix = m_weightMatrix;
    auto& systemMatrix = m_systemMatrix;

#ifdef OMP
    #pragma omp parallel for default(none) \
            shared(undeformedVertices, neighborhood, weightMatrix, systemMatrix)
#endif
    for (int i = 0; i < undeformedVertices.rows(); i++) { // Iterate over the vertices
        for (int neighbor : neighborhood[i]) { // Iterate over the neighbors
            systemMatrix(i, i) += weightMatrix(i, neighbor);
            systemMatrix(i, neighbor) -= weightMatrix(i, neighbor);
        }
    }
}

void Arap::precomputeDeformation(Eigen::MatrixXd& vertices, Eigen::MatrixXi& faces) {
    // Copy the vertices to a new matrix before any deformation operation
    m_undeformedVertices = safeReplicate(vertices);

    populateNeighborhood(faces); // Neighborhood
    initializeWeightMatrix(faces); // Weights
    computeSystemMatrix(); // LHS
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
    std::vector<Eigen::Matrix3d> rotationMatrices(m_undeformedVertices.rows());

    auto& undeformedVertices = m_undeformedVertices;
    auto& neighborhood = m_neighborhood;
    auto& weightMatrix = m_weightMatrix;

#ifdef OMP
    #pragma omp parallel for default(none) \
            shared(undeformedVertices, neighborhood, weightMatrix, deformedVertices, rotationMatrices)
#endif
    for (int i = 0; i < undeformedVertices.rows(); i++) { // Iterate over the vertices
        const long numNeighbors = (long) (neighborhood[i].size());

        // The definitions for the matrices P, D and P_prime can be found in the paper!
        Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, numNeighbors);
        Eigen::MatrixXd D = Eigen::MatrixXd::Zero(numNeighbors, numNeighbors);
        Eigen::MatrixXd P_prime = Eigen::MatrixXd::Zero(3, numNeighbors);

        for (int j = 0; j < numNeighbors; j++) { // Iterate over the neighbors
            P.col(j) = undeformedVertices.row(i) - undeformedVertices.row(neighborhood[i][j]);
            D(j, j) = weightMatrix(i, neighborhood[i][j]);
            P_prime.col(j) = deformedVertices.row(i) - deformedVertices.row(neighborhood[i][j]);
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
        Eigen::Matrix3d R = U * I * V.transpose();
        rotationMatrices[i] = R;
    }

    return rotationMatrices;
}

Eigen::MatrixXd Arap::computeRHS(std::vector<Eigen::Matrix3d> rotationMatrices) {
    Eigen::MatrixXd rhs = Eigen::MatrixXd::Zero(m_undeformedVertices.rows(), 3);

    auto& undeformedVertices = m_undeformedVertices;
    auto& neighborhood = m_neighborhood;
    auto& weightMatrix = m_weightMatrix;
    auto& fixedVertices = m_fixedVertices;

#ifdef OMP
    #pragma omp parallel for default(none) \
            shared(undeformedVertices, neighborhood, weightMatrix, fixedVertices, rotationMatrices, rhs)
#endif
    for (int i = 0; i < undeformedVertices.rows(); i++) { // Iterate over the vertices
        Eigen::Vector3d rhsRow = Eigen::Vector3d(0.0, 0.0, 0.0);

        if (i == m_movingVertex) { // If the vertex is the moving one, get the new position
            rhsRow = m_movingVertexPosition;
        } else if (std::find(fixedVertices.begin(), fixedVertices.end(), i) != fixedVertices.end()) { // Current vertex is a fixed vertex
            rhsRow = undeformedVertices.row(i);
        } else { // Current vertex is not a fixed vertex but a to-be-deformed vertex
            for (int neighbor : neighborhood[i]) { // Iterate over the neighbors
                rhsRow += 0.5 * weightMatrix(i, neighbor) *
                          (undeformedVertices.row(i) - undeformedVertices.row(neighbor)) *
                          (rotationMatrices[i] + rotationMatrices[neighbor]);
            }
        }

        rhs.row(i) = rhsRow;
    }

    return rhs;
}

double Arap::computeRigidityEnergy(Eigen::MatrixXd& deformedVertices, std::vector<Eigen::Matrix3d> rotationMatrices) {
    double rigidityEnergy = 0.0; // rigidity energy

    auto& undeformedVertices = m_undeformedVertices;
    auto& neighborhood = m_neighborhood;
    auto& weightMatrix = m_weightMatrix;

#ifdef OMP
    #pragma omp parallel for default(none) \
            shared(undeformedVertices, neighborhood, weightMatrix, deformedVertices, rotationMatrices) \
            reduction(+: rigidityEnergy)
#endif
    for (int i = 0; i < undeformedVertices.rows(); i++) { // Iterate over the undeformed vertices
        double rigidityEnergyPerCell = 0.0; // energy per cell

        for (int neighbor : neighborhood[i]) { // Iterate over the neighbors
            Eigen::Vector3d deformedPositionsDiff = deformedVertices.row(i) - deformedVertices.row(neighbor);
            Eigen::Vector3d undeformedPositionsDiff = undeformedVertices.row(i) - undeformedVertices.row(neighbor);

            rigidityEnergyPerCell += weightMatrix(i, neighbor) * (deformedPositionsDiff - rotationMatrices[i] * undeformedPositionsDiff).squaredNorm();
        }

        rigidityEnergy += weightMatrix(i, i) * rigidityEnergyPerCell;
    }

    return rigidityEnergy;
}

Eigen::MatrixXd Arap::computeDeformation(Eigen::MatrixXd& currentVertices) {
    // Initial guess
    solver.compute(m_systemMatrix.sparseView());
    Eigen::MatrixXd deformedVertices = solver.solve(m_systemMatrix * currentVertices);

    // Start the timer
    const std::chrono::time_point<std::chrono::system_clock> t0 = std::chrono::system_clock::now();

    // Optimize over some iterations
    auto previousRigidityEnergy = DBL_MAX;
    for (int i = 0; i < MAX_NUM_ITERATIONS; i++) {
        // Estimate rotations
        std::vector<Eigen::Matrix3d> rotationMatrices = estimateRotations(deformedVertices);

        // Compute RHS
        Eigen::MatrixXd rhs = computeRHS(rotationMatrices);

        // Solve the system
        solver.compute(m_systemMatrix.sparseView());
        deformedVertices = solver.solve(rhs);

        // Performance analysis for each iteration
        const double rigidityEnergy = computeRigidityEnergy(deformedVertices, rotationMatrices);
        printf("Iteration %d: rigidity energy = %.4f\n", i, rigidityEnergy);

        // Stop early if the solution is good enough
        if (i >= MIN_NUM_ITERATIONS && abs(previousRigidityEnergy - rigidityEnergy) < LOWER_ENERGY_THRESHOLD) {
            printf("Iteration %d: Energy threshold %.4f reached! Stopping early..\n", i, LOWER_ENERGY_THRESHOLD);
            i = MAX_NUM_ITERATIONS;
        } else {
            previousRigidityEnergy = rigidityEnergy;
        }
    }

    // End the timer and print the duration
    const std::chrono::time_point<std::chrono::system_clock> t1 = std::chrono::system_clock::now();
    printf("Took %f seconds to deform..\n\n", std::chrono::duration<double>(t1 - t0).count());

    return deformedVertices;
}

