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
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

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

	if (shared.frames[shared.read_idx].empty()) {
		const int default_width = 640;
		const int default_height = 480;
		std::vector<unsigned char> empty_data(default_width * default_height * 3, 0);

		glTexImage2D(
			GL_TEXTURE_2D, 0, GL_RGB,
			default_width, default_height, 0,
			GL_BGR, GL_UNSIGNED_BYTE, empty_data.data()
		);
	}
	else {
		cv::Mat& current_frame = shared.frames[shared.read_idx];
		glTexImage2D(
			GL_TEXTURE_2D, 0, GL_RGB,
			current_frame.cols, current_frame.rows, 0,
			GL_BGR, GL_UNSIGNED_BYTE, current_frame.ptr()
		);
	}

	shared.processed = true;
	shared.cv.notify_all();

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

	// Posición
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)nullptr);
	glEnableVertexAttribArray(0);

	// TexCoords
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
	glEnableVertexAttribArray(1);

	glBindVertexArray(0);
}

void Renderer::initGL() {
	try {
		window = initWindow();
		initGLEW();
		configureOpenGL();

		bgShaderProgram = Shader(SHADER_ABSOLUTE_PATH + SHADER_PATH[0], SHADER_ABSOLUTE_PATH + SHADER_PATH[1]);
		bgShaderProgram.use();
		bgShaderProgram.set_int("backgroundTexture", 0);

		// DESPUÉS de creado el contexto GLEW/GLFW
		initBG();
		initBGQuad();
	} catch (const GLException& e) {
		std::cerr << "Error en initGL: " << e.what() << std::endl;
		glfwTerminate();
	}
}

void Renderer::updateBG() {
	std::lock_guard lock(shared.mut);

	if (!shared.frame_ready) return;

	shared.read_idx = 1 - shared.read_idx;
	shared.frame_ready = false;

	cv::Mat& current_frame = shared.frames[shared.read_idx];
	if (current_frame.empty() || current_frame.type() != CV_8UC3)
		return;

	glBindTexture(GL_TEXTURE_2D, shared.cam_texture);

	glTexSubImage2D(
		GL_TEXTURE_2D, 0, 0, 0,
		current_frame.cols, current_frame.rows,
		GL_BGR, GL_UNSIGNED_BYTE, current_frame.ptr()
	);

	glGenerateMipmap(GL_TEXTURE_2D);
}

void Renderer::renderBG() const {
	glDisable(GL_DEPTH_TEST);

	bgShaderProgram.use();

	glBindVertexArray(bgVAO);
	glActiveTexture(GL_TEXTURE0);
	
	{
		std::lock_guard lock(shared.mut);

		if (!shared.frame_ready) return;
		glBindTexture(GL_TEXTURE_2D, shared.cam_texture);
		shared.processed = true;

		shared.cv.notify_all();
	}

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
		/* lógica de reiniciar el objeto */
	}
}

void Renderer::run() {
    while (!glfwWindowShouldClose(window)) {
        processInput(window);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        updateBG();
        renderBG();
        
        drawAxes();
        glfwSwapBuffers(window);
        glfwPollEvents();
        checkGLError();
    }
}