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

void Renderer::renderBG() {
	glDisable(GL_DEPTH_TEST);

	glUseProgram(backgroundShaderProgram);

	{
		std::lock_guard lock(shared.mut);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, shared.cam_texture);
	}

	glBindVertexArray(quadVAO);
	glDrawArrays(GL_TRIANGLES, 0, 6);

	glEnable(GL_DEPTH_TEST);
}

void Renderer::renderObj() {

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

void drawSphere(float radius = 1.0f, int segments = 32) {
	glColor3f(0.8f, 0.8f, 0.2f);

	// Material properties for lighting
	GLfloat mat_ambient[] = { 0.7f, 0.7f, 0.2f, 1.0f };
	GLfloat mat_diffuse[] = { 0.8f, 0.8f, 0.2f, 1.0f };
	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);

	// Fixed sphere generation (corrected divisions)
	for (int i = 0; i <= segments; ++i) {
		float lat0 = glm::pi<float>() * (-0.5f + float(i - 1) / segments);
		float z0 = radius * sin(lat0);
		float zr0 = radius * cos(lat0);

		float lat1 = glm::pi<float>() * (-0.5f + float(i) / segments);
		float z1 = radius * sin(lat1);
		float zr1 = radius * cos(lat1);

		glBegin(GL_QUAD_STRIP);
		for (int j = 0; j <= segments; ++j) {
			float lng = 2 * glm::pi<float>() * float(j) / segments;
			float x = cos(lng);
			float y = sin(lng);

			glNormal3f(x * zr0, y * zr0, z0);
			glVertex3f(x * zr0, y * zr0, z0);
			glNormal3f(x * zr1, y * zr1, z1);
			glVertex3f(x * zr1, y * zr1, z1);
		}
		glEnd();
	}
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