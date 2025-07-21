#pragma once

#include <array>
#include <format>
#include <iostream>
#include <GLFW/glfw3.h>
#include <glm/ext/scalar_constants.hpp>
#include <string>

#include "shaders.h"
#include "shared_data.h"

const unsigned int WINDOW_WIDTH = 800;
const unsigned int WINDOW_HEIGHT = 600;

constexpr char const* TITLE = "prueba con esfera cambiar el nombre para producción";

class GLException : public std::runtime_error {
public:
	explicit GLException(const std::string& msg)
		: std::runtime_error(std::format("OpenGL: {}", msg)) {}
};

class Renderer {
	SharedData& shared;
	GLFWwindow* window;

	void initBG();
	void initBGQuad();
	void initGL();

	GLuint bgShaderProgram;
	GLuint bgVAO;
	GLuint bgVBO;
	void updateBG();
	void renderBG();

	GLuint objShaderProgram;
	GLuint objVAO;
	GLuint objVBO;
	void updateObj();
	void renderObj();

public:
	explicit Renderer(SharedData&);
	void run();
};