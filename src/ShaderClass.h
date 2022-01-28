#ifndef SHADER_CLASS_H
#define SHADER_CLASS_H

#include <cerrno>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "../utils/glad.h"


std::string get_file_contents(const char* filename);

class Shader {
public:
	unsigned int ID;
	Shader(const char* vertexPath,const char* fragmentPath);

	void use();

	void setBool(const std::string &name, bool value) const;
	void setInt(const std::string &name, int value) const;
	void setFloat(const std::string &name, float value) const;
	void setMat4(const std::string& name, glm::mat4& mat)const;
};


#endif