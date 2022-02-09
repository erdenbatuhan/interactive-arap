/**
 * Project: Interactive ARAP
 * File:    Arap-Ceres.cpp
 * Authors: Batuhan Erden, Cansu Yildirim, Anas Shahzad, Alexander Epple
 */

#include "../include/Arap-Ceres.h"

struct ARAPEnergy
{
    ARAPEnergy(
        int numNeighbors
    ) :
      _numNeighbors(numNeighbors)
    {
    }

    //------------------------------------------------
    // parameters:
    // pi = old cell point
    // pi_prime = new cell point
    // Ri = cell rotation
    // Multiple, for each neighbor:
    // wij = uniform / cotan weight
    // pj = old neighbor point
    // pj_prime = new neighbor point
    //------------------------------------------------
    template <typename T>
    bool operator()(
      T const *const *parameters,
      T *residuals
    ) const
    {
        const T *pi = parameters[0];
        const T *pi_prime = parameters[1];
        const T *rotation = parameters[2];

        T quat[4];
        ceres::RotationMatrixToQuaternion(rotation, quat);

        // Calculate for each neighbor
        for (int i = 0; i < _numNeighbors; i++)
        {
            const T &weight = parameters[3 + (i * 3)][0];
            const T *pj = parameters[4 + (i * 3)];
            const T *pj_prime = parameters[5 + (i * 3)];

            // Compute edge vectors
            const T eij[3] = {pi[0] - pj[0], pi[1] - pj[1], pi[2] - pj[2]};
            const T eij_prime[3] = {pi_prime[0] - pj_prime[0], pi_prime[1] - pj_prime[1], pi_prime[2] - pj_prime[2]};

            // Rotate old edge
            T eij_rot[3];
            ceres::QuaternionRotatePoint(quat, eij, eij_rot);

            // Subtract new - rotated
            T res[3] = {eij_prime[0] - eij_rot[0], eij_prime[1] - eij_rot[1], eij_prime[2] - eij_rot[2]};

            // Compute weighted residual
            residuals[0] += weight * ceres::sqrt(ceres::DotProduct(res, res));
        }

        return true;
    }

    static ceres::CostFunction *Create(
      int numNeighbors
    )
    {
        // TODO: Potentially play with stride for better performance
        auto energy = new ceres::DynamicAutoDiffCostFunction<ARAPEnergy, 4>(new ARAPEnergy(numNeighbors));

        // Old cell point
        energy->AddParameterBlock(3);
        // New cell point
        energy->AddParameterBlock(3);
        // Cell rotation
        energy->AddParameterBlock(9);

        // For each neighbor point
        for (int i = 0; i < numNeighbors; ++i)
        {
            // Uniform / cotan weight
            energy->AddParameterBlock(1);
            // Old neighbor point
            energy->AddParameterBlock(3);
            // New neighbor point
            energy->AddParameterBlock(3);
        }

        // Only one output: cell energy
        energy->SetNumResiduals(1);

        return energy;
    }

    int _numNeighbors;
};

void ArapCeres::updateMovingVertex(
  const int movingVertex,
  const Eigen::Vector3f &movingVertexPosition
)
{
    m_movingVertex = movingVertex;
    m_movingVertexPosition = movingVertexPosition.cast<double>();
}

std::vector<int> ArapCeres::collectFixedVertices(
  Eigen::MatrixXi &faces,
  const std::vector<int> &anchorFaces
) const
{
    std::vector<int> fixedVertices;

    // Add the vertices of each face to the fixed vertices
    for (int anchorFace : anchorFaces)
    {
        Eigen::VectorXi faceVertices = faces.row(anchorFace);

        for (int j = 0; j < faces.cols(); j++)
        {
            fixedVertices.push_back(faceVertices(j));
        }
    }

    // Add the selected vertex to the fixed vertices
    fixedVertices.push_back(m_movingVertex);

    return fixedVertices;
}

Eigen::MatrixXd ArapCeres::computeDeformation(
  Eigen::MatrixXd &vertices,
  Eigen::MatrixXi &faces,
  std::map<int, std::vector<int>> &neighborhood,
  const std::vector<int> &anchorFaceIds
)
{
    Eigen::MatrixXd_R verts = convert(vertices);
    verts.block<1,3>(m_movingVertex, 0) = m_movingVertexPosition;
    Eigen::MatrixXd_R dVerts = safeReplicate(verts);

    std::vector<int> fixedVertices = collectFixedVertices(faces, anchorFaceIds);

    ceres::Problem problem;

    // Initialize rotation matrix & weights storage
    std::vector<Eigen::Matrix3d> matrices(verts.rows());

    int totalWeights = 0;
    for(int i = 0; i < neighborhood.size(); ++i)
    {
      totalWeights += neighborhood[i].size();
    }

    std::vector<double> weights;
    weights.reserve(totalWeights);

    // Create vector of blocks for solver
    std::vector<std::vector<double *>> blocks(verts.rows());

    // Create blocks & energy terms
    for (Eigen::Index i = 0; i < verts.rows(); ++i)
    {
        std::vector<double*>& block = blocks[i];

        block.push_back(verts.row(i).data());
        block.push_back(dVerts.row(i).data());

        // Add rotation
        matrices[i] = Eigen::Matrix3d();
        matrices[i].setIdentity();
        block.push_back(matrices[i].data());

        // Add weights & neighbors
        auto neighbors = neighborhood[i];
        for (int j = 0; j < neighbors.size(); ++j)
        {
            // TODO: Currently uniform, replace with cotan weight
            weights.push_back(1.0);
            block.push_back(&weights.back());
            block.push_back(&verts(neighbors[j], 0));
            block.push_back(&dVerts(neighbors[j], 0));
        }

        // Create dynamic energy function
        ceres::CostFunction *energy = ARAPEnergy::Create(neighbors.size());

        // Add block
        problem.AddResidualBlock(energy, NULL, blocks[i]);
    }

    // Fix all vertices of undeformed mesh
    for (int i = 0; i < verts.rows(); ++i)
    {
        problem.SetParameterBlockConstant(verts.row(i).data());
    }

    // Fix anchor vertices in deformed mesh
    for (int i = 0; i < fixedVertices.size(); ++i)
    {
        problem.SetParameterBlockConstant(dVerts.row(i).data());
    }

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    // TODO: Play with these for best perfomance / quality
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.max_num_iterations = 50;
    options.parameter_tolerance = 1e-8;
    options.num_threads = 8;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    return verts;
}
