#include <math.h>
#include <iostream>

#include <gl/glew.h>
#include <GLFW/glfw3.h>
#include <ceres/ceres.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "include/Eigen.h"
#include "include/shaderClass.h"
#include "include/myarap.h"


struct vertData {
	glm::vec3 currVert;
	std::vector<int> nei;
};

struct helper3d {
	std::vector<float> verti;
	std::vector<unsigned int> indi;
	std::vector<int> anchor_points;

	//contains all the vertices, their neighbours and their colors
	std::vector<vertData> model_det;
	
};
helper3d bunny_model;

void readFile3D(std::string filename1);
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
void processInput(GLFWwindow* window);


// helper functions to change color
void changeColor(glm::vec3 curr_cor);
void changeColor_ind(int chind);
std::vector<int> NNfind(glm::vec3 curr_cor,int noNei);
std::vector<int> sortArr(std::vector<float> arr);
std::vector<int> connection_Finds(glm::vec3 curr_cor, int curr_ind, int max_nei);
glm::vec3 intersectPoint(glm::vec3 rayVector, glm::vec3  rayPoint, glm::vec3 planeNormal, glm::vec3 planePoint);
void findIntersec(glm::vec3 rayPoint,glm::vec3 raynorm);
void printvec3(glm::vec3 arr1) {
	for (int i = 0; i < 3;i++) {
		std::cout << arr1[i] << "_";
	}
	std::cout << std::endl;
}
void printvec4(glm::vec4 arr1) {
	for (int i = 0; i < 3; i++) {
		std::cout << arr1[i] << "_";
	}
	std::cout <<std::endl;
}



// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

//camera stuff
float aroundy = 90.0;
glm::vec3 camerapos = glm::vec3(5.0f, 0.0f, 0.0f);
glm::vec3 frontpos = glm::vec3(0.0f, 0.0f, 0.0f);
glm::vec3 uppos = glm::vec3(0.0f, 1.0f, 0.0f);
float radius = glm::length(camerapos);
glm::mat4 globalpers;
glm::mat4 globalview;
glm::mat4 globalmodel;

// anchor selection threshold
float globalthresAnchor = 0.02;


std::vector<glm::vec3> ScreenPosToWorldRay(
	int mouseX, int mouseY,int screenWidth, int screenHeight,glm::mat4 ViewMatrix,glm::mat4 ProjectionMatrix      
) {

	glm::vec3 out_origin;
	glm::vec3 out_direction;
	glm::vec4 lRayStart_NDC(((float)mouseX / (float)screenWidth - 0.5f) * 2.0f, ((float)mouseY / (float)screenHeight - 0.5f) * 2.0f, -1.0, 1.0f);
	glm::vec4 lRayEnd_NDC(((float)mouseX / (float)screenWidth - 0.5f) * 2.0f,((float)mouseY / (float)screenHeight - 0.5f) * 2.0f,0.0,1.0f);


	glm::mat4 InverseProjectionMatrix = glm::inverse(ProjectionMatrix);

	// The View Matrix goes from World Space to Camera Space.
	glm::mat4 InverseViewMatrix = glm::inverse(ViewMatrix);

	glm::vec4 lRayStart_camera = InverseProjectionMatrix * lRayStart_NDC;    lRayStart_camera /= lRayStart_camera.w;
	glm::vec4 lRayStart_world = InverseViewMatrix * lRayStart_camera; lRayStart_world /= lRayStart_world.w;
	glm::vec4 lRayEnd_camera = InverseProjectionMatrix * lRayEnd_NDC;      lRayEnd_camera /= lRayEnd_camera.w;
	glm::vec4 lRayEnd_world = InverseViewMatrix * lRayEnd_camera;   lRayEnd_world /= lRayEnd_world.w;

	glm::vec3 lRayDir_world(lRayEnd_world - lRayStart_world);
	lRayDir_world = glm::normalize(lRayDir_world);


	out_origin = glm::vec3(lRayStart_world);
	out_direction = glm::normalize(lRayDir_world);
	std::vector<glm::vec3> resi;
	resi.push_back(out_origin);
	resi.push_back(out_direction);
	return resi;
}




int main()
{


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


//#######################################################	
//#######################################################
	// glfw: initialize and configure
	// ------------------------------
	if (!glfwInit())
	{
		std::cerr << "Could not init glfw!" << std::endl;
		exit(-1);
	}
	else
	{
		// Set all the required options for GLFW
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
		glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif
	}

	// glfw window creation
	// --------------------
	GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Interactive ARAP", NULL, NULL);
	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}
	else
	{
		glfwMakeContextCurrent(window);
		glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
		glfwSetMouseButtonCallback(window,mouse_button_callback);
	}

	// Initialize OpenGL
	glewExperimental = GL_TRUE;
	if (GLEW_OK != glewInit())
	{
		std::cout << "Failed to initialize GLEW" << std::endl;
		exit(-1);
	}

//#######################################################	
//#######################################################


	
	//reading the file
	Shader ourShader("resources/shaders/model.vert", "resources/shaders/model.frag");
	readFile3D("resources/models/bunny.off");
	int n_vert = bunny_model.verti.size();
	int n_indi = bunny_model.indi.size();

	
	//rending functions not important
	unsigned int VBO, VAO, EBO;
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);




	// render loop
	// -----------
	while (!glfwWindowShouldClose(window))
	{


		// Rendering functions not important
		glBindVertexArray(VAO);
		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * bunny_model.verti.size(), &bunny_model.verti.front(), GL_STATIC_DRAW);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * bunny_model.indi.size(), &bunny_model.indi.front(), GL_STATIC_DRAW);
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
		globalpers = projection;
		glm::mat4 view = glm::mat4(1.0f); 
		view = glm::lookAt(camerapos, frontpos, uppos);
		ourShader.setMat4("view", view);
		globalview = view;
		glm::mat4 model_modeli = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, 0.0f));
		ourShader.setMat4("model", model_modeli);
		globalmodel = model_modeli;
		
		ourShader.use();
		glBindVertexArray(VAO); 
		glDrawElements(GL_TRIANGLES, bunny_model.indi.size(), GL_UNSIGNED_INT, 0);
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	glDeleteVertexArrays(1, &VAO);
	glDeleteBuffers(1, &VBO);
	
	glfwTerminate();

//##########################################################################
//################ ARAP IMPLEMENTATION STARTS HERE #########################
//##########################################################################
	std::cout << "Now implementing the ARAP method" << std::endl;
	//MYARAP currARAP(bunny_model.verti,bunny_model.indi,bunny_model.anchor_points);

//##########################################################################
//##########################################################################
	

	return 0;
}

std::vector<int> sortArr(std::vector<float> arr)
{
	int n = arr.size();
	// Vector to store element
	// with respective present index
	std::vector<std::pair<float, int> > vp;

	// Inserting element in pair vector
	// to keep track of previous indexes
	for (int i = 0; i < n; ++i) {
		vp.push_back(std::make_pair(arr[i], i));
	}

	// Sorting pair vector
	sort(vp.begin(), vp.end());

	std::vector<int> resi;
	for (int i = 0; i < vp.size(); i++) {
		resi.push_back(vp[i].second);
	}
	return resi;
}

std::vector<int> NNfind(glm::vec3 curr_cor, int noNei) {

	std::vector<float> dister;

	for (int i = 0; i < bunny_model.model_det.size(); i++) {
		dister.push_back(glm::distance(bunny_model.model_det[i].currVert, curr_cor));
	}
	std::vector<int> temp3 = sortArr(dister);
	std::vector<int> res_nei(temp3.begin(), temp3.begin() + noNei);

	return res_nei;
}

std::vector<int> connection_Finds(glm::vec3 curr_cor,int curr_ind,int max_nei) {
	std::vector<int> conn_nei;

	for (int i = 0; i < int(bunny_model.indi.size()/3);i++) {
		int currtemp1 = (i * 3);
		if (conn_nei.size() > max_nei) {
			break;
		}
		else {
			for (int j = 0; j < 3;j++) {
				if (std::find(conn_nei.begin(), conn_nei.end(), bunny_model.indi[currtemp1 + j]) == conn_nei.end()) {
					conn_nei.push_back(bunny_model.indi[currtemp1 + j]);
				}
			}
		
		}
		//end 
	}
	for (int i = 0; i < conn_nei.size();i++) {
		std::cout << "daily deli " << conn_nei[i] << std::endl;
	}
	return conn_nei;
}

void changeColor(glm::vec3 curr_cor) {
	std::cout << "it came here" << curr_cor[0] << std::endl;
	int min_ind = 0;
	float min_dist = 100;
	
	for (int i = 0; i < bunny_model.model_det.size();i++) {
		if (glm::distance(curr_cor,bunny_model.model_det[i].currVert) < min_dist) {
			min_ind = i;
			min_dist = glm::distance(curr_cor, bunny_model.model_det[i].currVert);
		}
	}
	//now change the color of the neighbour 
	for (int i = 0; i < bunny_model.model_det[min_ind].nei.size();i++) {
		int currtemp = 6*(bunny_model.model_det[min_ind].nei[i]);
		bunny_model.verti[currtemp + 3] = 1.0;
		bunny_model.verti[currtemp + 4] = 0.0;
		bunny_model.verti[currtemp + 5] = 1.0;
		
	}


}

void processInput(GLFWwindow* window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);
	if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
		radius = glm::sqrt(glm::pow(camerapos[0], 2) + glm::pow(camerapos[2], 2));
		//std::cout << radius << std::endl;
		float camX = static_cast<float>(sin(glm::radians(aroundy)) * radius);
		float camZ = static_cast<float>(cos(glm::radians(aroundy)) * radius);
		aroundy = aroundy + 0.1;
		//std::cout << "right before" << camerapos[0] << "_" << camerapos[1] << "_" << camerapos[1] << std::endl;
		camerapos = glm::vec3(camX, camerapos[1], camZ);
		//std::cout << "rihgt after" << camerapos[0] << "_" << camerapos[1] << "_" << camerapos[1] << std::endl;
	}
	if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
		radius = glm::sqrt(glm::pow(camerapos[0], 2) + glm::pow(camerapos[2], 2));
		float camX = static_cast<float>(sin(glm::radians(aroundy)) * radius);
		float camZ = static_cast<float>(cos(glm::radians(aroundy)) * radius);
		aroundy = aroundy - 0.1;
		//std::cout << "left before" << camerapos[0] << "_" << camerapos[1] << "_" << camerapos[1] << std::endl;
		camerapos = glm::vec3(camX, camerapos[1], camZ);
		//std::cout << "left after" << camerapos[0] << "_" << camerapos[1] << "_" << camerapos[1] << std::endl;

	}
	if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
		radius = glm::sqrt(glm::pow(camerapos[0], 2) + glm::pow(camerapos[2], 2));
		//std::cout << radius << std::endl;
		//std::cout << "up before" << camerapos[0] << "_" << camerapos[1] << "_" << camerapos[1] << std::endl;
		camerapos = glm::vec3(camerapos[0], camerapos[1] + 0.025, camerapos[2]);
		//std::cout << "up after" << camerapos[0] << "_" << camerapos[1] << "_" << camerapos[1] << std::endl;

	}
	if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
		radius = glm::sqrt(glm::pow(camerapos[0], 2) + glm::pow(camerapos[2], 2));
		camerapos = glm::vec3(camerapos[0], camerapos[1] - 0.025, camerapos[2]);
	}
	if (glfwGetKey(window, GLFW_KEY_ENTER) == GLFW_PRESS) {
		glfwSetWindowShouldClose(window, true);
	}

}

glm::vec3 intersectPoint(glm::vec3 rayVector, glm::vec3  rayPoint, glm::vec3 planeNormal, glm::vec3 planePoint) {
	glm::vec3 diff = rayPoint - planePoint;
	float prod1 =  glm::dot(diff, planeNormal);
	float prod2 = glm::dot(rayVector,planeNormal);
	float prod3 = prod1 / prod2;
	return (rayPoint - rayVector * glm::vec3(prod3,prod3,prod3));
}
void changeColor_ind(int chind) {
	for (int i = 0; i < bunny_model.model_det[chind].nei.size(); i++) {
		int currtemp = 6 * (bunny_model.model_det[chind].nei[i]);
		bunny_model.verti[currtemp + 3] = 1.0;
		bunny_model.verti[currtemp + 4] = 0.0;
		bunny_model.verti[currtemp + 5] = 1.0;
	}
}
void findIntersec(glm::vec3 rayPoint, glm::vec3 raynorm) {

	glm::vec3 point1 = rayPoint;
	float y_sec = (-1*rayPoint[0] * raynorm[1] / raynorm[0]) + rayPoint[1];
	float z_sec = (-1 * rayPoint[0] * raynorm[2] / raynorm[0]) + rayPoint[2];
	glm::vec3 point2 = glm::vec3(0.0,y_sec,z_sec);
	
	
	float thresho = globalthresAnchor;

	std::vector<int> secinter;

	for (int i = 0; i < bunny_model.model_det.size();i++) {
		glm::vec4 new_cor = globalmodel * glm::vec4(bunny_model.model_det[i].currVert,1);
		float d = glm::length(glm::cross(point2 - point1,glm::vec3(new_cor[0], new_cor[1], new_cor[2]) - point1)) / glm::length(point2 - point1);
		if (d < thresho) {
			secinter.push_back(i);
		}
	}
	
	int ind_min = 0;
	float dist_min = 100.0;
	for (int i = 0; i < secinter.size();i++) {
		int currtemp1 = secinter[i];
		//std::cout << "....." << std::endl;
		glm::vec4 worldcor = globalmodel * glm::vec4(bunny_model.model_det[currtemp1].currVert, 1);
		//std::cout << "]]]]]]" << std::endl;
		float currdist = glm::distance(glm::vec3(worldcor[0], worldcor[1], worldcor[2]),camerapos);
		if (currdist < dist_min) {
			ind_min = i;
			dist_min = currdist;
		}
	}
	

	for (int i = 0; i < secinter.size(); i++) {
		int currtemp1 = secinter[i];
		int mintemp1 = secinter[ind_min];
		glm::vec4 worldcor = globalmodel*glm::vec4(bunny_model.model_det[currtemp1].currVert,1);
		glm::vec4 minvec = globalmodel * glm::vec4(bunny_model.model_det[mintemp1].currVert, 1);
		float currdist = glm::distance(worldcor,minvec);
		if (currdist < thresho) {
			changeColor_ind(currtemp1);
			bunny_model.anchor_points.push_back(currtemp1);
		}
	}


}
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
		double xpos, ypos;
		//getting cursor position
		glfwGetCursorPos(window, &xpos, &ypos);



		std::vector<glm::vec3> cam2World =ScreenPosToWorldRay(
			xpos, (SCR_HEIGHT-ypos),SCR_WIDTH, SCR_HEIGHT, globalview,globalpers);
		findIntersec(cam2World[0],cam2World[1]);
	
	}

		
}
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	// make sure the viewport matches the new window dimensions; note that width and 
	// height will be significantly larger than specified on retina displays.
	glViewport(0, 0, width, height);
}
void readFile3D(std::string filename1) {
	
	//reading the file
	std::vector<float> vertices_float;
	std::vector<unsigned int> indic_float;
	std::ifstream myStream(filename1);
	std::string temp1 ;
	std::string adder;
	
	int titli = 0;

	std::getline(myStream, temp1);
	


	std::getline(myStream, temp1);
	
	while (std::getline(myStream,temp1)) {
		std::stringstream test(temp1);
		std::vector<std::string> line_sep;
		while (std::getline(test, adder, ' '))
		{
			line_sep.push_back(adder);
		}

		if (line_sep.size()==4) {
			for (int i = 1; i < line_sep.size();i++) {
				bunny_model.indi.push_back(stoi(line_sep[i]));
			}

		}
		else if (line_sep.size()==7) {
			titli++;
			for (int i = 0; i < 3; i++) {
				bunny_model.verti.push_back(stof(line_sep[i]));
			}
			bunny_model.verti.push_back(float(255 / 255));
			bunny_model.verti.push_back(float(255/ 255));
			bunny_model.verti.push_back(float(0 / 255));
		}
		
	}

	//normalize the values now
	float min_max[] = {bunny_model.verti[0],bunny_model.verti[0] ,bunny_model.verti[1] ,bunny_model.verti[1] ,bunny_model.verti[2] ,bunny_model.verti[2]};

	for (int i = 0; i<int(bunny_model.verti.size()/6);i++) {
		int currtemp = (6 * i);
		glm::vec3 curr_vec = glm::vec3(bunny_model.verti[currtemp], bunny_model.verti[currtemp+1], bunny_model.verti[currtemp+2]);
		//x value
		if (curr_vec[0] < min_max[0]) {
			min_max[0] = curr_vec[0];
		}
		else if (curr_vec[0] > min_max[1]) {
			min_max[1] = curr_vec[0];
		}
		//y value
		if (curr_vec[1] < min_max[2]) {
			min_max[2] = curr_vec[1];
		}
		else if (curr_vec[1] > min_max[3]) {
			min_max[3] = curr_vec[1];
		}
		//z value
		if (curr_vec[2] < min_max[4]) {
			min_max[4] = curr_vec[2];
		}
		else if (curr_vec[2] > min_max[5]) {
			min_max[5] = curr_vec[2];
		}
	}

	// found the min_max. now normalizing
	for (int i = 0; i<int(bunny_model.verti.size() / 6); i++) {
		int currtemp = (6 * i);
		bunny_model.verti[currtemp] = (bunny_model.verti[currtemp] - min_max[0])/(min_max[1]-min_max[0]);
		bunny_model.verti[currtemp+1] = (bunny_model.verti[currtemp+1] - min_max[2]) / (min_max[3] - min_max[2]);
		bunny_model.verti[currtemp+2] = (bunny_model.verti[currtemp+2] - min_max[4]) / (min_max[5] - min_max[4]);
	}

	int neighbour_per = 20;

	for (int i = 0; i<int(bunny_model.verti.size() / 6); i++) {
		int currtemp = (6 * i);
		glm::vec3 curr_vec = glm::vec3(bunny_model.verti[currtemp], bunny_model.verti[currtemp + 1], bunny_model.verti[currtemp + 2]);
		vertData temp3;
		temp3.currVert = curr_vec;
		bunny_model.model_det.push_back(temp3);
	}

	for (int i = 0; i<int(bunny_model.verti.size() / 6); i++) {
		std::vector<int> res_nei = NNfind(bunny_model.model_det[i].currVert,neighbour_per);
		//std::vector<int> res_nei = connection_Finds(bunny_model.model_det[i].currVert, i, neighbour_per);
		bunny_model.model_det[i].nei = res_nei;
	}


}





