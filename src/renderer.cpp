#include "renderer.h"

void checkGLError() {
	if (GLenum err = glGetError(); err != GL_NO_ERROR)
		throw GLException(std::format("{} ", std::to_string(err)));
}

Renderer::Renderer(SharedData& s) : shared(s) {
	std::cout << "[renderer] RENDERER GENERATED\n INITIALIZING...\n";
	initGL();
	std::cout << "[renderer] RENDERER INITIALIZED\n";
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
	std::cout << "GLEW version: " << glewGetString(GLEW_VERSION) << "\n";
	GLint max_units = 0;
	glGetIntegerv(GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS, &max_units);
	std::cout << "Max texture units: " << max_units << "\n";
}

void configureOpenGL() {
	glEnable(GL_DEPTH_TEST);
	glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
	checkGLError();
}

void Renderer::initGL() {
	window = initWindow();
	std::cout << "[renderer] GLFW has been initialized successfuly\n";

	initGLEW();
	std::cout << "[renderer] GLEW has been initialized successfuly\n";

	configureOpenGL();

	glfwSwapInterval(1);
	glfwSetWindowSizeCallback(window, framebuffer_size_callback);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0.0, WINDOW_WIDTH, WINDOW_HEIGHT, 0.0, 0.0, 1.0);
	glMatrixMode(GL_MODELVIEW);

	int frameWidth, frameHeight;
	glfwGetFramebufferSize(window, &frameWidth, &frameHeight);
	glViewport(0, 0, frameWidth, frameHeight);

	updateBG();
	std::cout << "[renderer] Background has been initialized successfuly\n";
}


bool Renderer::updateBG() {
	std::cout << "[updateBg] " << shared.opengl_frames.size() << " left to eat\n";
	cv::Mat current_frame;

	if (!shared.opengl_frames.empty()) {
		std::cout << "[updateBg] yum\n";
		current_frame = shared.opengl_frames.front();
		shared.opengl_frames.pop();

		glGenTextures(1, &shared.cam_texture);
		glBindTexture(GL_TEXTURE_2D, shared.cam_texture);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

		GLenum inputFormat;
		switch (current_frame.channels())
		{
		case 1:
			inputFormat = GL_LUMINANCE;
			break;
		case 3:
			inputFormat = GL_BGR;
			break;
		case 4:
			inputFormat = GL_BGRA;
			break;
		default:
			throw GLException("Formato de imagen no soportado");
		}

		glTexImage2D(GL_TEXTURE_2D,
			0,
			GL_RGB,
			current_frame.cols,
			current_frame.rows,
			0,
			inputFormat,
			GL_UNSIGNED_BYTE,
			current_frame.ptr()
		);

		glGenerateMipmap(GL_TEXTURE_2D);

		return true;
	}
	else {
		std::cout << "[updateBg] Nothing to eat\n";
		return false;
	}
}

void Renderer::renderBG() {
	if (!updateBG()) return;

	glClearColor(0.1f, 0.1f, 0.1f, 0.f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);

	glEnable(GL_TEXTURE_2D);

	glBegin(GL_QUADS);
	glTexCoord2i(0, 0);
	glVertex2i(0, 0);
	glTexCoord2i(0, 1);
	glVertex2i(0, WINDOW_HEIGHT);
	glTexCoord2i(1, 1);
	glVertex2i(WINDOW_WIDTH, WINDOW_HEIGHT);
	glTexCoord2i(1, 0);
	glVertex2i(WINDOW_WIDTH, 0);

	glEnd();

	glDeleteTextures(1, &shared.cam_texture);
	glDisable(GL_TEXTURE_2D);
}

void Renderer::updateObj() {
	/*
	* 
	* función para update el obj
	* 
	*/
}

void Renderer::renderObj() {
	/*
	*
	* función para render el obj
	*
	*/
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
		/* lógica de reiniciar el objeto */
	}
}

void Renderer::run() {
	std::cout << "[renderLoop] Starting loop...\n";
    while (!glfwWindowShouldClose(window)) {
		std::cout << "[renderLoop] Beginning of loop\n";
		processInput(window);
		std::cout << "[renderLoop] Clearing buffers\n";
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        renderBG();
		drawAxes();
		drawSphere();
		std::cout << "[renderLoop] Background rendered\n";

        // drawAxes() (opengl antiguo) no es código no es código no es código
		std::cout << "[renderLoop] Swapping buffers\n";
		glfwSwapBuffers(window);
		std::cout << "[renderLoop] Polling events\n";
		glfwPollEvents();
        checkGLError();
		std::cout << "[renderLoop] End of loop\n";
	}
}