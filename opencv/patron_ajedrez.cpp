#include <opencv2/opencv.hpp>
#include <iostream>

std::pair<bool, cv::Point3f>  chess_center_point(cv::Mat frame, cv::Size patternSize, std::vector<cv::Point2f> corners) {
    cv::Mat gray;

    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    cv::Point3f center(0.f, 0.f, 0.f);

    bool found = cv::findChessboardCorners(
        gray, patternSize, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK
    );

    if (found) {
        cv::drawChessboardCorners(frame, patternSize, corners, found);

        if (!corners.empty()) {
            for (const auto& corner : corners) {
                center.x += corner.x;
                center.y += corner.y;
            }

            center.x /= (float)corners.size();
            center.y /= (float)corners.size();
        }
    }
    else{ center = {0.f, 0.f, 0.f}; }

    return std::make_pair(found, center);
}

int main() {
    cv::VideoCapture cap(0);

    if (!cap.isOpened()) {
        std::cerr << "Error: No se pudo abrir la cámara.\n";
        return -1;
    }

    const int chessboard_cols = 7;  // esquinas horizontales
    const int chessboard_rows = 7;  // esquinas verticales
    cv::Size patternSize(chessboard_cols, chessboard_rows);

    cv::Mat frame;
    cv::Mat gray;

    std::vector<cv::Point2f> corners;

    bool running = true;

    while (running) {
        cap >> frame;
        if (frame.empty()) break;

        auto chess_detect = chess_center_point(frame, patternSize, corners);

        if (chess_detect.first) cv::putText(frame, "Ajedrez detectado", { chess_detect.second.x, chess_detect.second.y }, cv::FONT_HERSHEY_SIMPLEX, 1.0, { 0, 255, 0 }, 2);

        cv::imshow("Detección de ajedrez", frame);
        if (cv::waitKey(1) == 27) running = false; // 27 -> Esc; 1 -> maxFPS = No limit FPS
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
