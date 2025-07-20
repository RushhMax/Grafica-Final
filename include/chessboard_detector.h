#pragma once
#include <array>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdlib.h>
#include <vector>

#include "camera.h"

constexpr short CHECKERBOARD_ROWS = 4;
constexpr short CHECKERBOARD_COLS = 7;

constexpr short MIN_FRAMES = 15;

class ChessboardDetector {
	Camera& camera;
	SharedData& shared;
	cv::TermCriteria crit;
	int flags = (
		cv::CALIB_CB_ADAPTIVE_THRESH |
		cv::CALIB_CB_NORMALIZE_IMAGE |
		cv::CALIB_CB_FAST_CHECK
	);
	cv::Mat camera_matrix;
	cv::Mat dist_coeffs;

	cv::Size pattern_size = cv::Size(CHECKERBOARD_COLS, CHECKERBOARD_ROWS);
	std::vector<cv::Point3f> generate_3d_object_points();
	void camera_calibration();
public:
	explicit ChessboardDetector(Camera&, SharedData&);
	void update();
};