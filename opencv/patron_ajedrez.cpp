#include <opencv2/opencv.hpp>
#include <iostream>

cv::Point3f chess_center_point(cv::Mat frame, cv::Size patternSize, std::vector<cv::Point2f> corners) {
    cv::Mat gray;

    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    cv::Point3f center(0.f, 0.f, 0.f);

    bool found = cv::findChessboardCorners(
        gray, patternSize, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE
    );

    if (found) {
        cv::drawChessboardCorners(frame, patternSize, corners, found);
        
        for (const auto& corner : corners) {
            center.x += corner.x;
            center.y += corner.y;
        }

        center.x /= (float)corners.size();
        center.y /= (float)corners.size();
    }

    return center;
}

int main() {
    cv::VideoCapture cap(0);

    if (!cap.isOpened()) {
        std::cerr << "Error: No se pudo abrir la cámara.\n";
        return -1;
    }

    const int chessboard_cols = 7;  // esquinas horizontales
    const int chessboard_rows = 6;  // esquinas verticales
    cv::Size patternSize(chessboard_cols, chessboard_rows);

    cv::Mat frame;
    cv::Mat gray;

    std::vector<cv::Point2f> corners;

    bool running = true;

    while (running) {
        cap >> frame;
        if (frame.empty()) break;

        auto center = chess_center_point(frame, patternSize, corners);

        cv::putText(frame, "Ajedrez detectado", { center.x, center.y }, cv::FONT_HERSHEY_SIMPLEX, 1.0, { 0, 255, 0 }, 2);

        cv::imshow("Detección de ajedrez", frame);
        if (cv::waitKey(33) == 27) running = false; // 27 -> Esc; 33 -> maxFPS = 30 FPS
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
