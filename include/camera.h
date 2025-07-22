#pragma once

#include <opencv2/opencv.hpp>

#include "shared_data.h"

class Camera {
	cv::VideoCapture cam;
	SharedData& shared;
public:
	explicit Camera(int, SharedData&);
	bool get_frame(cv::OutputArray frame);
	void set_frame();
};