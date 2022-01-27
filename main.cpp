#include <iostream>
#include "utils/glad.h"
#include "GLFW/glfw3.h"
#include "ceres/ceres.h"
#include <math.h>
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "src/ShaderClass.h"
#include "utils/stb.h"
#include "src/Arap.h"
#include<unistd.h>


struct vertexData {
	glm::vec3 currentVertex;
	std::vector<int> neighbor;
};

struct Helper3D {
	std::vector<float> vertices;
	std::vector<unsigned int> indices;
	std::vector<int> anchorPoints;

	//contains all the vertices, their neighbours and their colors
	std::vector<vertexData> modelDet;
	
};
Helper3D bunnyModel;

//TODO: Delete these after separating gui and main classes
void readFile3D(std::string fileName);
void framebufferSizeCallback(GLFWwindow* window, int width, int height);
void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
void processInput(GLFWwindow* window);
void changeColor(glm::vec3 currentCoord);
void changeColorWithIndex(int index);
void findIntersect(glm::vec3 rayPoint,glm::vec3 rayNorm);
void printVec3(glm::vec3 arr1);
void printVec4(glm::vec4 arr1);
std::vector<int> NNFind(glm::vec3 currentCoord,int noNeighbor);
std::vector<int> sortArr(std::vector<float> arr);
std::vector<int> findConnections(glm::vec3 currentCoord, int currentInd, int maxNeighbor);
glm::vec3 intersectPoint(glm::vec3 rayVector, glm::vec3  rayPoint, glm::vec3 planeNormal, glm::vec3 planePoint);


const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

//camera settings
float aroundAxisY = 90.0;
float globalThresholdAnchor = 0.02; // anchor selection threshold

glm::vec3 cameraPos = glm::vec3(5.0f, 0.0f, 0.0f);
glm::vec3 frontPos = glm::vec3(0.0f, 0.0f, 0.0f);
glm::vec3 upperPos = glm::vec3(0.0f, 1.0f, 0.0f);

float radius = glm::length(cameraPos);

glm::mat4 globalPerspect;
glm::mat4 globalView;
glm::mat4 globalModel;


void framebufferSizeCallback(GLFWwindow* window, int width, int height) {
    // Make sure the viewport matches the new window dimensions
    // Note: Width and height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

std::vector<glm::vec3> ScreenPosToWorldRay(int mouseX, int mouseY,int screenWidth, int screenHeight, glm::mat4 viewMatrix, glm::mat4 ProjectionMatrix) {
    glm::vec3 outOrigin;
    glm::vec3 outDirection;
    glm::vec4 lRayStart_NDC(((float)mouseX / (float)screenWidth - 0.5f) * 2.0f, ((float)mouseY / (float)screenHeight - 0.5f) * 2.0f, -1.0, 1.0f);
    glm::vec4 lRayEnd_NDC(((float)mouseX / (float)screenWidth - 0.5f) * 2.0f,((float)mouseY / (float)screenHeight - 0.5f) * 2.0f,0.0,1.0f);

    glm::mat4 InverseProjectionMatrix = glm::inverse(ProjectionMatrix);
    // The View Matrix goes from World Space to Camera Space.
    glm::mat4 InverseViewMatrix = glm::inverse(viewMatrix);

    glm::vec4 lRayStartCamera = InverseProjectionMatrix * lRayStart_NDC;
    glm::vec4 lRayStartWorld = InverseViewMatrix * lRayStartCamera;
    glm::vec4 lRayEndCamera = InverseProjectionMatrix * lRayEnd_NDC;
    glm::vec4 lRayEndWorld = InverseViewMatrix * lRayEndCamera;

    lRayStartCamera /= lRayStartCamera.w;
    lRayStartWorld /= lRayStartWorld.w;
    lRayEndCamera /= lRayEndCamera.w;
    lRayEndWorld /= lRayEndWorld.w;

    glm::vec3 lRayDirWorld(lRayEndWorld - lRayStartWorld);

    lRayDirWorld = glm::normalize(lRayDirWorld);

    outOrigin = glm::vec3(lRayStartWorld);
    outDirection = glm::normalize(lRayDirWorld);

    std::vector<glm::vec3> result;
    result.push_back(outOrigin);
    result.push_back(outDirection);

    return result;
}

void changeColorWithIndex(int index) {
    for (int i = 0; i < bunnyModel.modelDet[index].neighbor.size(); i++) {
        int currTemp = 6 * (bunnyModel.modelDet[index].neighbor[i]);

        bunnyModel.vertices[currTemp + 3] = 1.0;
        bunnyModel.vertices[currTemp + 4] = 0.0;
        bunnyModel.vertices[currTemp + 5] = 1.0;
    }
}

void findIntersect(glm::vec3 rayPoint, glm::vec3 rayNorm) {
    glm::vec3 point1 = rayPoint;

    float ySec = (-1 * rayPoint[0] * rayNorm[1] / rayNorm[0]) + rayPoint[1];
    float zSec = (-1 * rayPoint[0] * rayNorm[2] / rayNorm[0]) + rayPoint[2];

    glm::vec3 point2 = glm::vec3(0.0,ySec,zSec);

    float thresholdVal = globalThresholdAnchor;
    std::vector<int> secInter;

    for (int i = 0; i < bunnyModel.modelDet.size();i++) {
        glm::vec4 new_cor = globalModel * glm::vec4(bunnyModel.modelDet[i].currentVertex,1);
        float d = glm::length(glm::cross(point2 - point1,glm::vec3(new_cor[0], new_cor[1], new_cor[2]) - point1)) / glm::length(point2 - point1);
        if (d < thresholdVal) {
            secInter.push_back(i);
        }
    }

    int indMin = 0;
    float distMin = 100.0;

    for (int i = 0; i < secInter.size();i++) {
        int currentTemp = secInter[i];
        glm::vec4 worldCoord = globalModel * glm::vec4(bunnyModel.modelDet[currentTemp].currentVertex, 1);

        float currentDistance = glm::distance(glm::vec3(worldCoord[0], worldCoord[1], worldCoord[2]),cameraPos);
        if (currentDistance < distMin) {
            indMin = i;
            distMin = currentDistance;
        }
    }


    for (int i = 0; i < secInter.size(); i++) {
        int currentTemp = secInter[i];
        int minTemp = secInter[indMin];

        glm::vec4 worldCoord = globalModel*glm::vec4(bunnyModel.modelDet[currentTemp].currentVertex,1);
        glm::vec4 minVec = globalModel * glm::vec4(bunnyModel.modelDet[minTemp].currentVertex, 1);

        float currentDistance = glm::distance(worldCoord,minVec);
        if (currentDistance < thresholdVal) {
            changeColorWithIndex(currentTemp);
            bunnyModel.anchorPoints.push_back(currentTemp);
        }
    }
}

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        double xPos, yPos;

        glfwGetCursorPos(window, &xPos, &yPos);  //getting cursor position
        std::vector<glm::vec3> cam2World = ScreenPosToWorldRay(
                xPos, (SCR_HEIGHT-yPos),SCR_WIDTH, SCR_HEIGHT, globalView,globalPerspect);

        findIntersect(cam2World[0],cam2World[1]);
    }
}

std::vector<int> sortArr(std::vector<float> arr) {
    int n = arr.size();
    // Vector to store element with respective present index
    std::vector<std::pair<float, int> > vp;

    // Inserting element in pair vector to keep track of previous indexes
    for (int i = 0; i < n; ++i) {
        vp.push_back(std::make_pair(arr[i], i));
    }

    sort(vp.begin(), vp.end());  // Sorting pair vector

    std::vector<int> result;
    for (int i = 0; i < vp.size(); i++) {
        result.push_back(vp[i].second);
    }
    return result;
}

std::vector<int> NNFind(glm::vec3 currentCoord, int noNeighbor) {
    std::vector<float> dister;

    for (int i = 0; i < bunnyModel.modelDet.size(); i++) {
        dister.push_back(glm::distance(bunnyModel.modelDet[i].currentVertex, currentCoord));
    }

    std::vector<int> temp3 = sortArr(dister);
    std::vector<int> resultNeighbor(temp3.begin(), temp3.begin() + noNeighbor);

    return resultNeighbor;
}

void readFile3D(std::string fileName) {
    std::cout << "Reading the file: " << fileName <<'\n';

    std::vector<float> vertices_float;
    std::vector<unsigned int> indicesFloat;

    std::ifstream myStream(fileName);
    std::string temp1 ;
    std::string adder;

    int t = 0;

    std::getline(myStream, temp1);
    std::getline(myStream, temp1);

    while (std::getline(myStream,temp1)) {
        std::stringstream test(temp1);
        std::vector<std::string> lineSep;

        while (std::getline(test, adder, ' ')) {
            lineSep.push_back(adder);
        }

        if (lineSep.size() == 4) {
            for (int i = 1; i < lineSep.size(); i++) {
                bunnyModel.indices.push_back(stoi(lineSep[i]));
            }
        } else if (lineSep.size() == 7) {
            t++;
            for (int i = 0; i < 3; i++) {
                bunnyModel.vertices.push_back(stof(lineSep[i]));
            }
            bunnyModel.vertices.push_back(float(255 / 255));
            bunnyModel.vertices.push_back(float(255/ 255));
            bunnyModel.vertices.push_back(float(0 / 255));
        }
    }

    //normalize the values now
    std::cout << "indices size " << bunnyModel.indices.size()<< '\n';
    std::cout << "vertices size " << bunnyModel.vertices.size()<< '\n';

    float minMax[] = {bunnyModel.vertices[0],bunnyModel.vertices[0] ,bunnyModel.vertices[1] ,bunnyModel.vertices[1] ,bunnyModel.vertices[2] ,bunnyModel.vertices[2]};
    std::cout << "min max found " << '\n';

    std::cout << bunnyModel.vertices[0] << '\n';
    std::cout << bunnyModel.vertices[1] << '\n';
    std::cout << bunnyModel.vertices[2] << '\n';

    for (int i = 0; i < int(bunnyModel.vertices.size() / 6); i++) {
        int currTemp = (6 * i);
        glm::vec3 currentVec = glm::vec3(bunnyModel.vertices[currTemp], bunnyModel.vertices[currTemp+1], bunnyModel.vertices[currTemp+2]);

        //x value
        if (currentVec[0] < minMax[0]) {
            minMax[0] = currentVec[0];
        } else if (currentVec[0] > minMax[1]) {
            minMax[1] = currentVec[0];
        }
        //y value
        if (currentVec[1] < minMax[2]) {
            minMax[2] = currentVec[1];
        } else if (currentVec[1] > minMax[3]) {
            minMax[3] = currentVec[1];
        }
        //z value
        if (currentVec[2] < minMax[4]) {
            minMax[4] = currentVec[2];
        } else if (currentVec[2] > minMax[5]) {
            minMax[5] = currentVec[2];
        }
    }

    // found the minMax. now normalizing
    for (int i = 0; i < int(bunnyModel.vertices.size() / 6); i++) {
        int currTemp = (6 * i);

        bunnyModel.vertices[currTemp] = (bunnyModel.vertices[currTemp] - minMax[0]) / (minMax[1] - minMax[0]);
        bunnyModel.vertices[currTemp + 1] = (bunnyModel.vertices[currTemp + 1] - minMax[2]) / (minMax[3] - minMax[2]);
        bunnyModel.vertices[currTemp + 2] = (bunnyModel.vertices[currTemp + 2] - minMax[4]) / (minMax[5] - minMax[4]);
    }

    int neighbourPer = 20;

    for (int i = 0; i < int(bunnyModel.vertices.size() / 6); i++) {
        int currTemp = (6 * i);

        glm::vec3 currentVec = glm::vec3(bunnyModel.vertices[currTemp], bunnyModel.vertices[currTemp + 1], bunnyModel.vertices[currTemp + 2]);
        vertexData temp3;

        temp3.currentVertex = currentVec;
        bunnyModel.modelDet.push_back(temp3);
    }

    for (int i = 0; i < int(bunnyModel.vertices.size() / 6); i++) {
        std::vector<int> resultNeighbor = NNFind(bunnyModel.modelDet[i].currentVertex,neighbourPer);
        //std::vector<int> resultNeighbor = findConnections(bunnyModel.modelDet[i].currentVertex, i, neighbourPer);

        bunnyModel.modelDet[i].neighbor = resultNeighbor;
    }

}

void processInput(GLFWwindow* window) {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, true);
    }

    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
        radius = glm::sqrt(glm::pow(cameraPos[0], 2) + glm::pow(cameraPos[2], 2));

        float camX = static_cast<float>(sin(glm::radians(aroundAxisY)) * radius);
        float camZ = static_cast<float>(cos(glm::radians(aroundAxisY)) * radius);

        aroundAxisY = aroundAxisY + 0.1;
        cameraPos = glm::vec3(camX, cameraPos[1], camZ);
    }

    if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
        radius = glm::sqrt(glm::pow(cameraPos[0], 2) + glm::pow(cameraPos[2], 2));

        float camX = static_cast<float>(sin(glm::radians(aroundAxisY)) * radius);
        float camZ = static_cast<float>(cos(glm::radians(aroundAxisY)) * radius);

        aroundAxisY = aroundAxisY - 0.1;
        cameraPos = glm::vec3(camX, cameraPos[1], camZ);
    }

    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
        radius = glm::sqrt(glm::pow(cameraPos[0], 2) + glm::pow(cameraPos[2], 2));

        cameraPos = glm::vec3(cameraPos[0], cameraPos[1] + 0.025, cameraPos[2]);
    }

    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
        radius = glm::sqrt(glm::pow(cameraPos[0], 2) + glm::pow(cameraPos[2], 2));

        cameraPos = glm::vec3(cameraPos[0], cameraPos[1] - 0.025, cameraPos[2]);
    }

    if (glfwGetKey(window, GLFW_KEY_ENTER) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, true);
    }
}

std::vector<int> findConnections(glm::vec3 currentCoord, int currentInd, int maxNeighbor) {
	std::vector<int> connectedNeighbor;

	for (int i = 0; i < int (bunnyModel.indices.size() / 3); i++) {
		int currentTemp = (i * 3);

		if (connectedNeighbor.size() > maxNeighbor) {
			break;
		} else {
			for (int j = 0; j < 3;j++) {
				if (std::find(connectedNeighbor.begin(), connectedNeighbor.end(), bunnyModel.indices[currentTemp + j]) == connectedNeighbor.end()) {
					connectedNeighbor.push_back(bunnyModel.indices[currentTemp + j]);
				}
			}
		}
	}
	return connectedNeighbor;
}

void changeColor(glm::vec3 currentCoord) {
	int minInd = 0;
	float minDist = 100;
	
	for (int i = 0; i < bunnyModel.modelDet.size();i++) {
		if (glm::distance(currentCoord,bunnyModel.modelDet[i].currentVertex) < minDist) {
			minInd = i;
			minDist = glm::distance(currentCoord, bunnyModel.modelDet[i].currentVertex);
		}
	}
	// Change the color of the neighbour
	for (int i = 0; i < bunnyModel.modelDet[minInd].neighbor.size();i++) {
		int currTemp = 6*(bunnyModel.modelDet[minInd].neighbor[i]);

		bunnyModel.vertices[currTemp + 3] = 1.0;
		bunnyModel.vertices[currTemp + 4] = 0.0;
		bunnyModel.vertices[currTemp + 5] = 1.0;
		
	}
}

glm::vec3 intersectPoint(glm::vec3 rayVector, glm::vec3 rayPoint, glm::vec3 planeNormal, glm::vec3 planePoint) {
	glm::vec3 diff = rayPoint - planePoint;

	float prod1 =  glm::dot(diff, planeNormal);
	float prod2 = glm::dot(rayVector,planeNormal);
	float prod3 = prod1 / prod2;

	return (rayPoint - rayVector * glm::vec3(
            prod3,
            prod3,
            prod3));
}

void printVec3(glm::vec3 arr1) {
    for (int i = 0; i < 3; i++) {
        std::cout << arr1[i] << "_";
    }
    std::cout << std::endl;
}

void printVec4(glm::vec4 arr1) {
    for (int i = 0; i < 3; i++) {
        std::cout << arr1[i] << "_";
    }
    std::cout <<std::endl;
}

int main() {
    char cur[100];
    std::cout << "Current path is " << getcwd(cur, 100) << '\n';

    chdir("../");
    std::cout << "Now, current path is " << getcwd(cur, 100) << '\n';

    std::cout << "################################################" << std::endl;
    std::cout << "######### Welcome to Interactive Arap ##########" << std::endl;
    std::cout << "################################################" << std::endl;
    std::cout << "Please choose the ARAP method you would like to perform : " << std::endl;
    std::cout << "1. Simple ARAP" << std::endl;
    std::cout << "2. Skeleton ARAP" << std::endl;
    std::cout << "3. Cage ARAP" << std::endl;

    std::string method1;
    std::cin >> method1;
    std::cout << "################# Initializing #################" << std::endl;
    std::cout << "Please choose the anchor points. Press 1 to enter press 0 to close" << std::endl;
    std::string mode1;
    std::cin >> mode1;
    if (mode1 == "0") {
        return 0;
    }

    // glfw: initialize and configure
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // glfw window creation
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
    if (window == NULL) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
    glfwSetMouseButtonCallback(window,mouseButtonCallback);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    //reading the file
    Shader ourShader("resources/shader/model.vert", "resources/shader/model.frag");
    readFile3D("data/bunny.off");

    int n_vert = bunnyModel.vertices.size();
    int n_indices = bunnyModel.indices.size();

    //rending functions not important
    unsigned int VBO, VAO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    // render loop
    while (!glfwWindowShouldClose(window)) {
        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float) * bunnyModel.vertices.size(), &bunnyModel.vertices.front(), GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * bunnyModel.indices.size(), &bunnyModel.indices.front(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);

        // checks for any keyboard or mouse input
        processInput(window);

        glEnable(GL_DEPTH_TEST);
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // updating the camera parameters
        glm::mat4 projection = glm::perspective(glm::radians(45.0f), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        ourShader.setMat4("projection", projection);
        globalPerspect = projection;
        glm::mat4 view = glm::mat4(1.0f);
        view = glm::lookAt(cameraPos, frontPos, upperPos);
        ourShader.setMat4("view", view);
        globalView = view;
        glm::mat4 modelMat = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, 0.0f));
        ourShader.setMat4("model", modelMat);
        globalModel = modelMat;

        ourShader.use();
        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, bunnyModel.indices.size(), GL_UNSIGNED_INT, 0);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);

    glfwTerminate();

//################ ARAP IMPLEMENTATION STARTS HERE #########################
    std::cout << "Now implementing the ARAP method" << std::endl;
    //MyArap currARAP(bunnyModel.vertices,bunnyModel.indices,bunnyModel.anchorPoints);

//##########################################################################

    return 0;
}




