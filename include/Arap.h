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

class Arap
{
public:
  Arap() = default;
  ~Arap() = default;

  virtual void updateMovingVertex(int, const Eigen::Vector3f&) = 0;

  virtual std::vector<int> collectFixedVertices(Eigen::MatrixXi&, const std::vector<int>&) const = 0;
  virtual Eigen::MatrixXd computeDeformation(Eigen::MatrixXd&, Eigen::MatrixXi&,
    std::map<int, std::vector<int>>&, const std::vector<int>&) = 0;

protected:
  // The current moving vertex
  int m_movingVertex{};
  Eigen::Vector3d m_movingVertexPosition{};
};

#endif // _ARAP_H_
