#pragma once

#include <mutex>
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <opencv2/opencv.hpp>

struct SharedData {
	std::mutex mut;
	bool running = true;

	cv::Mat frame;
	GLuint cam_texture;

	glm::mat4 chessboard_pos;

	// una vez que implementen la mano se pone lo que debería llevar
	struct Hand {
		glm::vec3 pos;
		bool detected = false;
		std::string status = "";
	} hand;
};