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

class Arap {
public:
    Arap();
    ~Arap() = default;

    void updateParameters(int);

    std::vector<int> collectFixedVertices(Eigen::MatrixXd&, std::vector<int>&) const;
    void runDeformation(Eigen::MatrixXd&, Eigen::MatrixXd&, std::vector<int>&, std::map<int, std::vector<int>>&) const;
private:
    // Neighborhood of vertices (Mapping between vertex id and its neighbor ids)
    const std::map<int, std::vector<int>> m_neighborhood;

    // The current moving vertex
    int m_movingVertex{};
};

#endif // _ARAP_H_
