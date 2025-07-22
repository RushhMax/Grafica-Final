#pragma once

// comprobación glew
#ifndef __gl_h_
#include <GL/glew.h>
#endif

#include <array>
#include <format>
#include <iostream>
#include <GLFW/glfw3.h>
#include <glm/ext/scalar_constants.hpp>
#include <string>

#include "shaders.h"
#include "shared_data.h"

const unsigned int WINDOW_WIDTH = 640;
const unsigned int WINDOW_HEIGHT = 480;

constexpr char const* TITLE = "Star Wars AR";

/*
* pares: vert
* impares: frag
* primeros dos para el background
* últimos dos para el 3d object
*/
const std::string SHADER_ABSOLUTE_PATH = "E:\\compgrafica\\resources\\";
const std::array<std::string, 4> SHADER_PATH = { "background.vert", "background.frag", "object.vert", "object.frag" };

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

	int last_width;
	int last_height;

	Shader bgShaderProgram;
	GLuint bgVAO;
	GLuint bgVBO;
	bool updateBG();
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