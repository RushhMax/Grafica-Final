#include "renderer.h"
#include "model_loader.hpp"
#include "texture_loader.hpp"
#include "shader_loader.hpp"

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

void calculateObjectDimensions(const std::vector<float>& vertices,
	float& minX, float& maxX,
	float& minY, float& maxY,
	float& minZ, float& maxZ,
	float& centroidX, float& centroidY, float& centroidZ) {
	if (vertices.empty()) return;

	minX = maxX = vertices[0];
	minY = maxY = vertices[1];
	minZ = maxZ = vertices[2];

	float sumX = 0, sumY = 0, sumZ = 0;
	size_t vertexCount = vertices.size() / 3;

	for (size_t i = 0; i < vertices.size(); i += 3) {
		float x = vertices[i];
		float y = vertices[i + 1];
		float z = vertices[i + 2];

		minX = std::min(minX, x);
		maxX = std::max(maxX, x);
		minY = std::min(minY, y);
		maxY = std::max(maxY, y);
		minZ = std::min(minZ, z);
		maxZ = std::max(maxZ, z);

		sumX += x;
		sumY += y;
		sumZ += z;
	}

	centroidX = sumX / vertexCount;
	centroidY = sumY / vertexCount;
	centroidZ = sumZ / vertexCount;
}

float calculateScaleFactor(float minX, float maxX, float minY, float maxY, float minZ, float maxZ) {
	float sizeX = maxX - minX;
	float sizeY = maxY - minY;
	float sizeZ = maxZ - minZ;

	float maxSize = std::max(sizeX, std::max(sizeY, sizeZ));
	return (maxSize > 0) ? 2.0f / maxSize : 1.0f;
}

void Renderer::renderBG() {
	if (!updateBG()) return;

	glDisable(GL_DEPTH_TEST);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0, WINDOW_WIDTH, 0, WINDOW_HEIGHT, -1, 1);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, shared.cam_texture);

	glColor3f(1.0f, 1.0f, 1.0f);

	glBegin(GL_QUADS);
	glTexCoord2i(0, 1); glVertex2i(0, 0);
	glTexCoord2i(0, 0); glVertex2i(0, WINDOW_HEIGHT);
	glTexCoord2i(1, 0); glVertex2i(WINDOW_WIDTH, WINDOW_HEIGHT);
	glTexCoord2i(1, 1); glVertex2i(WINDOW_WIDTH, 0);
	glEnd();

	glDeleteTextures(1, &shared.cam_texture);
	glDisable(GL_TEXTURE_2D);

	glPopMatrix(); // MODELVIEW
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);

	glEnable(GL_DEPTH_TEST);
}

void Renderer::updateObj() {
    if (!loadOBJ("E:\\compgrafica\\modelos\\star-war\\anlustarwars.obj", vertices)) {
        throw GLException("No se pudo cargar el modelo OBJ.");
    }

    objTexture = loadTexture("E:\\compgrafica\\modelos\\star-war\\anlustarwars.jpg");

    glm::vec3 min(FLT_MAX), max(-FLT_MAX);
    for (size_t i = 0; i < vertices.size(); i += 5) {
        glm::vec3 v(vertices[i], vertices[i + 1], vertices[i + 2]);
        min = glm::min(min, v);
        max = glm::max(max, v);
    }
    centroModelo = (min + max) * 0.5f;

	glEnable(GL_TEXTURE_2D);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
}


void Renderer::renderObj() {
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, WINDOW_WIDTH / float(WINDOW_HEIGHT), 0.1, 1000.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glLoadMatrixf(glm::value_ptr(shared.chessboard_pose));

	float minX, maxX, minY, maxY, minZ, maxZ;
	calculateObjectDimensions(vertices, minX, maxX, minY, maxY, minZ, maxZ,
		centroModelo.x, centroModelo.y, centroModelo.z);

	float scaleFactor = calculateScaleFactor(minX, maxX, minY, maxY, minZ, maxZ);

	float currentFrame = glfwGetTime();
	deltaTime = currentFrame - lastFrame;
	lastFrame = currentFrame;

	glm::vec3 pos;
	float angle = 0.0f;

	if (volverAlCentro) {
		estadoAnimacion = SUBIENDO;
		altura = 0.0f;
		avanceZ = 0.0f;
		pos = -centroModelo;
	}
	else if (activarAnimacion) {
		if (estadoAnimacion == SUBIENDO) {
			altura += velocidadSubida * deltaTime;
			if (altura >= 50.0f) {
				altura = 50.0f;
				estadoAnimacion = AVANZANDO;
			}
			pos = glm::vec3(0.0f, altura, 0.0f) - centroModelo;
		}
		else if (estadoAnimacion == AVANZANDO) {
			avanceZ -= velocidadAvance * deltaTime;
			if (avanceZ <= -100.0f) {
				avanceZ = -100.0f;
				estadoAnimacion = GIRANDO;
			}
			pos = glm::vec3(0.0f, altura, avanceZ) - centroModelo;
		}
		else if (estadoAnimacion == GIRANDO) {
			float tiempo = glfwGetTime();
			float radio = 100.0f;
			float velocidad = 1.0f;

			float x = radio * cos(velocidad * tiempo);
			float z = avanceZ + radio * sin(velocidad * tiempo);
			float y = altura + 2.0f * sin(velocidad * tiempo * 2.0f);

			pos = glm::vec3(x, y, z) - centroModelo;
			angle = tiempo * 180.0f / 3.14159f;  // en grados
		}
	}
	else {
		pos = -centroModelo;
	}

	// Aplicar transformaciones
	glTranslatef(pos.x, pos.y, pos.z);
	if (estadoAnimacion == GIRANDO)
		glRotatef(angle, 0.0f, 1.0f, 0.0f);

	glBindTexture(GL_TEXTURE_2D, objTexture);

	glScalef(scaleFactor, scaleFactor, scaleFactor);
	glTranslatef(-centroModelo.x, -centroModelo.y, -centroModelo.z);

	glBegin(GL_TRIANGLES);
	for (size_t i = 0; i < vertices.size(); i += 5) {
		float x = vertices[i];
		float y = vertices[i + 1];
		float z = vertices[i + 2];
		float u = vertices[i + 3];
		float v = vertices[i + 4];

		glTexCoord2f(u, v);
		glVertex3f(x, y, z);
	}
	glEnd();
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
}


/*
* 
*	FUNCIONES DRAW DE GLFW/GLEW FREEGLUT NO ERES MALO PERO GLFW >>>>>>>>>>>>
* 
*/

void Renderer::drawAxes(float len) {
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluPerspective(45.0, WINDOW_WIDTH / float(WINDOW_HEIGHT), 0.1, 100.0);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glLoadMatrixf(glm::value_ptr(shared.chessboard_pose));

	glEnable(GL_DEPTH_TEST);
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

	glPopMatrix(); // MODELVIEW
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW); // Siempre regresar a MODELVIEW al final
	glDisable(GL_DEPTH_TEST);
}


void Renderer::drawSphere(float radius, int segments) {
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, WINDOW_WIDTH / float(WINDOW_HEIGHT), 0.1, 100.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glLoadMatrixf(glm::value_ptr(shared.chessboard_pose));

	glColor3f(0.8f, 0.8f, 0.2f);

	glEnable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING); // Desactiva luces si estás solo coloreando

	for (int i = 0; i <= segments; ++i) {
		float lat0 = glm::pi<float>() * (-0.5f + float(i - 1) / segments);
		float z0 = radius * sin(lat0);
		float zr0 = radius * cos(lat0);

		float lat1 = glm::pi<float>() * (-0.5f + float(i) / segments);
		float z1 = radius * sin(lat1);
		float zr1 = radius * cos(lat1);

		glBegin(GL_QUAD_STRIP);
		for (int j = 0; j <= segments; ++j) {
			float lng = 2.0f * glm::pi<float>() * float(j) / segments;
			float x = cos(lng);
			float y = sin(lng);

			glVertex3f(x * zr0, y * zr0, z0);
			glVertex3f(x * zr1, y * zr1, z1);
		}
		glEnd();
	}
	glDisable(GL_DEPTH_TEST);
}


void processInput(GLFWwindow* window) {
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);
}

void Renderer::run() {
	//std::cout << "[renderLoop] Loading 3D object...\n";
    updateObj();
	std::cout << "[renderLoop] Starting loop...\n";
	while (!glfwWindowShouldClose(window)) {
		processInput(window);
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        renderBG();
		//drawAxes(5.0f);
		// drawSphere();

        renderObj();
		
		// drawAxes();

		// drawSphere(5.0f, 20);

		glfwSwapBuffers(window);
		glfwPollEvents();
        checkGLError();
        std::this_thread::sleep_for(FRAME_RATE);
	}
}