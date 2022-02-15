/**
 * Project: Interactive ARAP
 * File:    Arap-Ceres.cpp
 * Authors: Batuhan Erden, Cansu Yildirim, Anas Shahzad, Alexander Epple
 */

#include "../include/Arap-Ceres.h"

// Fitting term
struct FitEnergy
{
	FitEnergy(
		const Eigen::Vector3d& constraint,
		double weight
	) :
		_constraint(constraint),
		_weight(weight)
	{
	}

	template <typename T>
	bool operator()(
		const T* const position,
		T* residuals
		) const
	{
		residuals[0] = (position[0] - T(_constraint[0])) * T(_weight);
		residuals[1] = (position[1] - T(_constraint[1])) * T(_weight);
		residuals[2] = (position[2] - T(_constraint[2])) * T(_weight);
		return true;
	}

	static ceres::CostFunction* Create(
		const Eigen::Vector3d& constraint,
		double weight
	)
	{
		return new ceres::AutoDiffCostFunction<FitEnergy, 3, 3>(
			new FitEnergy(constraint, weight));
	}

	Eigen::Vector3d _constraint;
	double _weight;
};

// Regularization term
struct RegEnergy
{
	RegEnergy(
		const Eigen::Vector3d& edge,
		double weight
	) :
		_edge(edge),
		_weight(weight)
	{
	}

	template <typename T>
	bool operator()(
		const T* const pi,
		const T* const pj,
		const T* const rot,
		T* residuals
		) const
	{

		const T edge[3] =
		{
			T(_edge.x()),
			T(_edge.y()),
			T(_edge.z())
		};

		T edge_r[3] =
		{
			T(0),
			T(0),
			T(0)
		};

		ceres::AngleAxisRotatePoint(rot, edge, edge_r);

		const T edge_p[3] =
		{
			pi[0] - pj[0],
			pi[1] - pj[1],
			pi[2] - pj[2]
		};

		residuals[0] = T(_weight) * (edge_p[0] - edge_r[0]);
		residuals[1] = T(_weight) * (edge_p[1] - edge_r[1]);
		residuals[2] = T(_weight) * (edge_p[2] - edge_r[2]);

		return true;
	}

	static ceres::CostFunction* Create(
		const Eigen::Vector3d& edge,
		double weight
	)
	{
		return new ceres::AutoDiffCostFunction<RegEnergy, 3, 3, 3, 3>(
			new RegEnergy(edge, weight));
	}

	Eigen::Vector3d _edge;
	double _weight;
};

void ArapCeres::updateMovingVertex(
	const int movingVertex,
	const Eigen::Vector3f& movingVertexPosition
)
{
	m_movingVertex = movingVertex;
	m_movingVertexPosition = movingVertexPosition.cast<double>();
}

std::vector<int> ArapCeres::collectFixedVertices(
	Eigen::MatrixXi& faces,
	const std::vector<int>& anchorFaces
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
	Eigen::MatrixXd& vertices,
	Eigen::MatrixXi& faces,
	std::map<int, std::vector<int>>& neighborhood,
	const std::vector<int>& anchorFaceIds
)
{
	ceres::Problem problem;

	std::vector<int> fixedVertices = collectFixedVertices(faces, anchorFaceIds);

	// Original mesh
	Eigen::MatrixXd_R verts = convert(vertices);
	// Deformed mesh
	Eigen::MatrixXd_R dVerts = safeReplicate(verts);
	// Constraints
	Eigen::MatrixXd_R cVerts = Eigen::MatrixXd_R::Zero(fixedVertices.size(), 3);
	// Rotations
	Eigen::MatrixXd_R rotations = Eigen::MatrixXd_R::Zero(verts.rows(), 3);

	// Initialize weights
	int totalWeights = 0;
	std::vector<double> weights;
	for (int i = 0; i < neighborhood.size(); ++i)
	{
		totalWeights += neighborhood[i].size();
	}
	weights.reserve(totalWeights);

	// Add fixed vertices
	for(size_t i = 0; i < fixedVertices.size(); ++i)
	{
		auto idx = fixedVertices[i];
		cVerts.row(i) = verts.row(idx);

		ceres::CostFunction* fit = FitEnergy::Create(cVerts.row(i), 2.0);
		problem.AddResidualBlock(fit, NULL, dVerts.row(idx).data());
	}

	// Add moving vertex
	ceres::CostFunction* fitMoving = FitEnergy::Create(m_movingVertexPosition, 2.0);
	problem.AddResidualBlock(fitMoving, NULL, dVerts.row(m_movingVertex).data());

	// Add cells
	for (size_t i = 0; i < neighborhood.size(); ++i)
	{
		for (size_t j = 0; j < neighborhood[i].size(); ++j)
		{
			auto neighbor = neighborhood[i][j];
			Eigen::Vector3d eij = verts.row(i) - verts.row(neighbor);
			ceres::CostFunction* reg = RegEnergy::Create(eij, 1.0);
			problem.AddResidualBlock(reg, NULL, dVerts.row(i).data(), dVerts.row(neighbor).data(), rotations.row(i).data());
		}
	}

	ceres::Solver::Options options;
#if DEBUG
	options.minimizer_progress_to_stdout = true;
#endif
	options.linear_solver_type = ceres::LinearSolverType::CGNR;
	options.max_num_iterations = 10000;
	options.function_tolerance = 0.05;
	options.gradient_tolerance = 1e-4 * options.function_tolerance;
	options.num_threads = 4;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	std::cout << summary.FullReport() << "\n";

	return dVerts;
}
