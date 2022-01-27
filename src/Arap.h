#ifndef INTERACTIVE_ARAP_ARAP_H
#define INTERACTIVE_ARAP_ARAP_H

#include <iostream>
#include "../utils/glad.h"
#include "GLFW/glfw3.h"
#include "../utils/Eigen.h"
#include <ceres/ceres.h>
#include <math.h>
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"


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

#endif //INTERACTIVE_ARAP_ARAP_H