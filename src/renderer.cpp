#include <GL/glew.h>

#include "renderer.h"

void checkGLError() {
	GLenum err;
	std::string errorMessages;
	while ((err = glGetError()) != GL_NO_ERROR)
		errorMessages += std::to_string(err);
	if (!errorMessages.empty())
		throw GLException(errorMessages);
}

Renderer::Renderer(SharedData& s) : shared(s) {
	initGL();
}
/*
* 
*	FUNCIONES INIT DE GLEW/GLFW FREEGLUT ZZZZZZZZZZZZZZZZZZZZZZZ
* 
*/

void framebuffer_size_callback(GLFWwindow* /*window*/, int width, int height) {
	glViewport(0, 0, width, height);
}

GLFWwindow* initWindow() {
	if (!glfwInit()) throw GLException("No se pudo iniciar GLFW");

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);

	GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, TITLE, nullptr, nullptr);
	if (!window) {
		glfwTerminate();
		throw GLException("GLFW no pudo crear una ventana");
	}

	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

	return window;
}

void initGLEW() {
	glewExperimental = GL_TRUE;
	if (glewInit() != GLEW_OK) throw GLException("No se pudo iniciar GLEW");
}

void configureOpenGL() {
	glEnable(GL_DEPTH_TEST);
	glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
	checkGLError();
}

void Renderer::initGL() {
	try {
		window = initWindow();
		initGLEW();
		configureOpenGL();
	} catch (const GLException& e) {
		std::cerr << "Error en Renderer: " << e.what() << std::endl;
		glfwTerminate();
	}
}

void Renderer::renderBG()
{
}

void Renderer::renderObj()
{
}

/*
* 
*	FUNCIONES DRAW DE GLFW/GLEW FREEGLUT NO ERES MALO PERO GLFW >>>>>>>>>>>>
* 
*/

void drawAxes(float len = 1.5f) {
	glDisable(GL_LIGHTING);

	glBegin(GL_LINES);
	// x
	glColor3f(1.0f, 0.0f, 0.0f);

	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(len, 0.0f, 0.0f);

	// y
	glColor3f(0.0f, 1.0f, 0.0f);

	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, len, 0.0f);

	// z
	glColor3f(0.0f, 0.0f, 1.0f);

	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, len);
	glEnd();

	glEnable(GL_LIGHTING);
}

void processInput(GLFWwindow* window) {
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);

	// esto depende de que la lógica de reinicio se haga en el Renderer
	if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS) {
		/* TODO: reiniciar el objeto */
	}
}

void Renderer::run() {
	while (!glfwWindowShouldClose(window)) {
		try {
			processInput(window);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			{
				std::lock_guard lock(shared.mut);
				// TODO: update texture función en shared y todo eso
			}

			drawAxes();

			renderBG();
			renderObj();

			glfwSwapBuffers(window);
			glfwPollEvents();
			checkGLError();
		} catch (const GLException& e) {
			std::cerr << "Error en Renderer: " << e.what() << std::endl;
			glfwTerminate();
		}
	}
}