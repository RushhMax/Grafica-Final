#include "chessboard_detector.h"

ChessboardDetector::ChessboardDetector(Camera& c, SharedData& sd) : camera(c), shared(sd) {
    camera_calibration();
}

std::vector<cv::Point3f> ChessboardDetector::generate_3d_object_points() const {
    std::vector<cv::Point3f> obj_points_it;

    for (int i = 0; i < pattern_size.height; ++i)
        for (int j = 0; j < pattern_size.width; ++j)
            obj_points_it.emplace_back(j, i, 0.0f);

    return obj_points_it;
}

void ChessboardDetector::camera_calibration() {
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;

    std::vector<cv::Point3f> obj_points_it = generate_3d_object_points();

    cv::Mat frame;
    cv::Mat gray;
    cv::Mat small;
    while (image_points.size() < MIN_FRAMES) {
        camera.get_frame(frame);
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::resize(gray, small, cv::Size(), 0.5, 0.5);

        std::vector<cv::Point2f> corner_pts;
        std::vector<cv::Point2f> corners_small;

        cv::putText(frame, "Calibración en proceso...", { 25, 25 },
            cv::FONT_HERSHEY_SIMPLEX, 0.65, { 0, 255, 0 }, 2);
        cv::putText(frame, std::format("Capturas: {}", image_points.size()), { 25, frame.rows - 25 },
            cv::FONT_HERSHEY_SIMPLEX, 0.65, { 0, 255, 0 }, 2);

        int key = cv::waitKey(30);
        if (bool found = cv::findChessboardCorners(small, pattern_size, corners_small, flags); found) {
            if (bool detected = cv::findChessboardCorners(gray, pattern_size, corner_pts, flags);
                detected) {

                cv::drawChessboardCorners(frame, pattern_size, corner_pts, detected);
                
                cv::putText(frame, "Presione 'S' para tomar una muestra", { 25, 50 },
                    cv::FONT_HERSHEY_SIMPLEX, 0.65, { 0, 255, 0 }, 2);
                
                if (key == 's' || key == 'S') {
                    cv::cornerSubPix(
                        gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1),
                        crit
                    );
                    object_points.push_back(obj_points_it);
                    image_points.push_back(corner_pts);
                }
            }
        }

        cv::imshow("Calibrando...", frame);
        if (key == 27) abort();
    }

    cv::destroyAllWindows();

    if (image_points.size() < MIN_FRAMES)
        CV_Error(cv::Error::StsVecLengthErr, "Frames insuficientes para calibrar");

    std::vector<cv::Mat> __rvecs; // descarte
    std::vector<cv::Mat> __tvecs; // descarte

    double error = cv::calibrateCamera(
        object_points, image_points, frame.size(),
        camera_matrix, dist_coeffs,
        __rvecs, __tvecs
    );

    std::cout << "Error RMS en calibración: " << error << " pixeles" << std::endl; // your order is confirmed
}

/*
* 
*   FUNCIONES TRANSFORMACIÓN OPENCV -> GLM
* 
*/

glm::mat3 cv_rvec_to_glm(const cv::Mat& rvec) {
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    return glm::mat3(
        R.at<double>(0, 0), -R.at<double>(1, 0), -R.at<double>(2, 0),
        R.at<double>(0, 1), -R.at<double>(1, 1), -R.at<double>(2, 1),
        R.at<double>(0, 2), -R.at<double>(1, 2), -R.at<double>(2, 2)
    );
}

glm::vec3 cv_tvec_to_glm(const cv::Mat& tvec) {
    return glm::vec3(
        tvec.at<double>(0),
        -tvec.at<double>(1),
        -tvec.at<double>(2)
    );
}

// para mover el objeto con respecto al mundo
glm::mat4 get_glm_model_mat(const cv::Mat& rvec, const cv::Mat& tvec) {
    glm::mat3 R = cv_rvec_to_glm(rvec);
    glm::vec3 t = cv_tvec_to_glm(tvec);

    glm::mat4 model(R);
    model[3] = glm::vec4(t, 1.0f);

    return glm::inverse(model);
}

// para mover el mundo
glm::mat4 get_glm_view_mat() {
    return glm::mat4(1.0f); // no se va a mover el mundo
}

glm::mat4 get_glm_projection_mat(const cv::Mat& camera_matrix,
    float width, float height,
    float near_plane = 0.1f,
    float far_plane = 100.0f) {

    const float fx = camera_matrix.at<float>(0, 0);
    const float fy = camera_matrix.at<float>(1, 1);
    const float cx = camera_matrix.at<float>(0, 2);
    const float cy = camera_matrix.at<float>(1, 2);

    const float left = -cx * near_plane / fx;
    const float right = (width - cx) * near_plane / fx;
    const float bottom = (cy - height) * near_plane / fy;
    const float top = cy * near_plane / fy;

    return glm::frustum(left, right, bottom, top, near_plane, far_plane);
}

/*
* 
*   UPDATE FUNCION UPDATE FUNCION UPDATE FUNCION UPDATE FUNCION UPDATE FUNCION UPDATE FUNCION UPDATE
* 
*/

void ChessboardDetector::update() {
    std::cout << "[chessboard_detector] " << shared.opencv_frames.size() << " left to eat\n";
    cv::Mat current_frame;

    if (!shared.opencv_frames.empty()) {
        current_frame = shared.opencv_frames.front();
        std::cout << "[renderer] yum\n";
        shared.opencv_frames.pop();
    }

    if (!current_frame.empty()) {
        cv::Mat gray;
        cv::cvtColor(current_frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corner_pts;
        bool detected = cv::findChessboardCorners(
            gray, pattern_size, corner_pts, flags);

        if (detected) {
            cv::cornerSubPix(
                gray, corner_pts, cv::Size(11, 11),
                cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

            auto object_points = generate_3d_object_points();
            cv::Mat rvec;
            cv::Mat tvec;
            cv::solvePnP(object_points, corner_pts,
                camera_matrix, dist_coeffs,
                rvec, tvec);

            shared.chessboard_pose = get_glm_model_mat(rvec, tvec);
        }
    }
}