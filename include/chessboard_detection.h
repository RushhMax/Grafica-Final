#pragma once
#include <array>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

constexpr short CHECKERBOARD_ROWS = 4;
constexpr short CHECKERBOARD_COLS = 7;

void camera_calibration(cv::VideoCapture&, std::vector<cv::Mat>&, std::vector<cv::Mat>&);

