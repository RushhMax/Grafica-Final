#pragma once

#ifndef __gl_h_
#include <GL/glew.h>
#endif

#include <array>
#include <mutex>
#include <glm/glm.hpp>
#include <opencv2/opencv.hpp>
#include <queue>

struct SharedData {
    bool running = true;

    // frames destinados a ser consumidos por cvThread
    std::queue<cv::Mat> opencv_frames;
    // frames destinados a ser consumidos por renderLoop
    std::queue<cv::Mat> opengl_frames;

    GLuint cam_texture = 0;
    glm::mat4 chessboard_pose;

    ~SharedData() {
        running = false;
    }
};