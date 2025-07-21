#pragma once

#ifndef __gl_h_
#include <GL/glew.h>
#endif

#include <array>
#include <mutex>
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <opencv2/opencv.hpp>

struct SharedData {
    std::mutex mut;
    std::condition_variable cv;
    bool frame_ready = false;
    bool processed = true;
    std::atomic<bool> running = true;

    std::array<cv::Mat, 2> frames;
    std::atomic<int> read_idx = 0;
    GLuint cam_texture = 0;
    glm::mat4 chessboard_pos;

    ~SharedData() {
        running = false;
        cv.notify_all();
    }
};