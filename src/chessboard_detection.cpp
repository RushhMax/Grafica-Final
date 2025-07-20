#include "chessboard_detection.h"

void camera_calibration(
    cv::VideoCapture& cam,
    cv::Mat& camera_matrix,
    cv::Mat& dist_coeffs,
    std::vector<cv::Mat>& rvecs,
    std::vector<cv::Mat>& tvecs,
    int min_frames = 15,
    cv::Size pattern_size = cv::Size(CHECKERBOARD_COLS, CHECKERBOARD_ROWS)
) {
    if (!cam.isOpened())
        CV_Error(cv::Error::StsBadArg, "La cámara no ha sido inicializada");

    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;

    std::vector<cv::Point3f> obj_points_it;
    for (int i = 0; i < pattern_size.height; ++i)
        for (int j = 0; j < pattern_size.width; ++j)
            obj_points_it.emplace_back(j, i, 0);

    cv::Mat frame;
    while (image_points.size() < min_frames) {
        cam >> frame;
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corner_pts;
        if (bool detected = cv::findChessboardCorners(
            gray,
            pattern_size,
            corner_pts,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK
        ); detected) {
            cv::cornerSubPix(
                gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001)
            );

            cv::drawChessboardCorners(frame, pattern_size, corner_pts, detected);
            cv::putText(frame, "Calibración en proceso...\nMantenga la cámara en el tablero", { 25, 25 }, cv::FONT_HERSHEY_SIMPLEX, 1.0, { 0, 255, 0 }, 2);
            object_points.push_back(obj_points_it);
            image_points.push_back(corner_pts);
        }

        cv::imshow("Calibrando...", frame);
        if (cv::waitKey(30) == 27) break;
    }

    cv::destroyAllWindows();

    if (image_points.size() < 5)
        CV_Error(cv::Error::StsVecLengthErr, "Frames insuficientes para calibrar");

    double error = cv::calibrateCamera(
        object_points, image_points, frame.size(),
        camera_matrix, dist_coeffs, rvecs, tvecs
    );

    std::cout << "Error RMS en calibración: " << error << " pixeles" << std::endl;
}