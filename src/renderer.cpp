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

void Renderer::initBG() {
	std::lock_guard lock(shared.mut);

	glGenTextures(1, &shared.cam_texture);
	glBindTexture(GL_TEXTURE_2D, shared.cam_texture);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT);

	glTexImage2D(
		GL_TEXTURE_2D, 0, GL_RGB,
		shared.frame.cols, shared.frame.rows, 0,
		GL_BGR, GL_UNSIGNED_BYTE, shared.frame.ptr()
	);

	glGenerateMipmap(GL_TEXTURE_2D);
}

void Renderer::initBGQuad() {
	std::array<float, 24> quadVertices = {
		// positions    // texCoords
		-1.0f,  1.0f,   0.0f, 1.0f,  // top-left
		-1.0f, -1.0f,   0.0f, 0.0f,  // bottom-left
		 1.0f, -1.0f,   1.0f, 0.0f,  // bottom-right

		-1.0f,  1.0f,   0.0f, 1.0f,  // top-left
		 1.0f, -1.0f,   1.0f, 0.0f,  // bottom-right
		 1.0f,  1.0f,   1.0f, 1.0f   // top-right
	};

	glGenVertexArrays(1, &bgVAO);
	glGenBuffers(1, &bgVBO);

	glBindVertexArray(bgVAO);
	glBindBuffer(GL_ARRAY_BUFFER, bgVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), quadVertices.data(), GL_STATIC_DRAW);

	// Posición (x, y)
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)nullptr);
	glEnableVertexAttribArray(0);

	// TexCoords (u, v)
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
	glEnableVertexAttribArray(1);

	glBindVertexArray(0);
}

void Renderer::initGL() {
	try {
		window = initWindow();
		initGLEW();
		configureOpenGL();

		// DESPUÉS de creado el contexto GLEW/GLFW
		initBG();
		initBGQuad();
	} catch (const GLException& e) {
		std::cerr << "Error en Renderer: " << e.what() << std::endl;
		glfwTerminate();
	}
}

void Renderer::updateBG() {
	std::lock_guard lock(shared.mut);

	if (shared.frame.empty() || shared.frame.type() != CV_8UC3)
		return;

	glBindTexture(GL_TEXTURE_2D, shared.cam_texture);

	glTexSubImage2D(
		GL_TEXTURE_2D, 0, 0, 0,
		shared.frame.cols, shared.frame.rows,
		GL_BGR, GL_UNSIGNED_BYTE, shared.frame.ptr()
	);

	glGenerateMipmap(GL_TEXTURE_2D);
}

void Renderer::renderBG() {
	glDisable(GL_DEPTH_TEST);

	glUseProgram(bgShaderProgram);

	{
		std::lock_guard lock(shared.mut);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, shared.cam_texture);
	}

	glBindVertexArray(bgVAO);
	glDrawArrays(GL_TRIANGLES, 0, 6);

	glEnable(GL_DEPTH_TEST);
}

void Renderer::updateObj() {

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

	std::array<GLfloat, 4> mat_ambient = { 0.7f, 0.7f, 0.2f, 1.0f };
	std::array<GLfloat, 4> mat_diffuse = { 0.8f, 0.8f, 0.2f, 1.0f };
	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient.data());
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse.data());

	for (int i = 0; i <= segments; ++i) {
		float lat0 = glm::pi<float>() * (-0.5f + float(i - 1 / segments));
		float z0 = radius * sin(lat0);
		float zr0 = radius * cos(lat0);

		float lat1 = glm::pi<float>() * (-0.5f + float(i / segments));
		float z1 = radius * sin(lat1);
		float zr1 = radius * cos(lat1);

		glBegin(GL_QUAD_STRIP);
		for (int j = 0; j <= segments; ++j) {
			float lng = 2 * glm::pi<float>() * float(j / segments);
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