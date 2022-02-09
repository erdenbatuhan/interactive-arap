/**
 * Project: Interactive ARAP
 * File:    Arap-Ceres.h
 * Authors: Batuhan Erden, Cansu Yildirim, Anas Shahzad, Alexander Epple
 */

#ifndef _ARAP_CERES_H_
#define _ARAP_CERES_H_

#include "Eigen.h"

#include <map>
#include <vector>
#include <utility>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

class Arap
{
public:
  Arap();
  ~Arap() = default;

  void updateMovingVertex(int, const Eigen::Vector3f &);

  std::vector<int> collectFixedVertices(Eigen::MatrixXi &, const std::vector<int> &) const;
  Eigen::MatrixXd computeDeformation(Eigen::MatrixXd &, Eigen::MatrixXi &,
                                     std::map<int, std::vector<int>> &, const std::vector<int> &);

private:
     // The current moving vertex
    int m_movingVertex{};
    Eigen::Vector3d m_movingVertexPosition{};
};

#endif // _ARAP_CERES_H_
