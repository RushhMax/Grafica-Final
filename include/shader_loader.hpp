#ifndef SHADER_LOADER_HPP
#define SHADER_LOADER_HPP

#include <string>
#include <GLFW/glfw3.h> 

GLuint loadShaderProgram(const std::string& vertexPath, const std::string& fragmentPath);

#endif