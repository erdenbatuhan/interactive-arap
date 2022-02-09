/**
 * Project: Interactive ARAP
 * File:    Arap-Ceres.h
 * Authors: Batuhan Erden, Cansu Yildirim, Anas Shahzad, Alexander Epple
 */

#ifndef _ARAP_CERES_H_
#define _ARAP_CERES_H_

#include "Arap.h"

#ifdef WIN32
#pragma push_macro("ERROR")
#undef ERROR
#endif

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#ifdef WIN32
#pragma pop_macro("ERROR")
#endif

class ArapCeres : public Arap
{
public:
  ArapCeres() = default;
  ~ArapCeres() = default;

  virtual void updateMovingVertex(int, const Eigen::Vector3f &) override;

  virtual std::vector<int> collectFixedVertices(Eigen::MatrixXi &, const std::vector<int> &) const override;
  virtual Eigen::MatrixXd computeDeformation(Eigen::MatrixXd &, Eigen::MatrixXi &,
      std::map<int, std::vector<int>> &, const std::vector<int> &) override;
};

#endif // _ARAP_CERES_H_
