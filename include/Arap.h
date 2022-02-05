#ifndef _ARAP_H_
#define _ARAP_H_

#include <cmath>
#include <iostream>

#include <gl/glew.h>
#include <GLFW/glfw3.h>
#include <ceres/ceres.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "Eigen.h"

class Arap {
public:
    unsigned int ID;
    std::string moder;
    std::vector<glm::vec3> vert;
    std::vector<int> ind;
    std::vector<int> anchors;

    Arap(const std::string currMode);

    void use(std::vector<glm::vec3> vertices,std::vector<unsigned int> ind,std::vector<int> anchorPoints);
    //void simpleARAP() const;
    //void skeletonARAP() const;
    //void cageARAP() const;
};

#endif // _ARAP_H_
