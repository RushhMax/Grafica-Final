#pragma once

// comprobacion glew
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
* �ltimos dos para el 3d object
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

	GLuint objTexture;
	GLuint objVAO;
	GLuint objVBO;
	std::vector<float> vertices;

	glm::vec3 centroModelo;

	enum EstadoAnimacion { SUBIENDO, AVANZANDO, GIRANDO };
	EstadoAnimacion estadoAnimacion = SUBIENDO;

	float avanceZ = 0.0f;
	float velocidadAvance = 30.0f;
	float altura = 0.0f;
	float velocidadSubida = 10.0f;
	bool activarAnimacion = false;
	bool volverAlCentro = false;
	float deltaTime = 0.0f;
	float lastFrame = 0.0f;

	Shader objShaderProgram;
	void updateObj();
	void renderObj();

public:
	explicit Renderer(SharedData&);
	void run();
};