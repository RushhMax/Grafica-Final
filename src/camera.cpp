#include "camera.h"

Camera::Camera(int device_id, SharedData& s) : shared(s) {
	cam.open(device_id);
	if (!cam.isOpened()) CV_Error(cv::Error::StsObjectNotFound, "No se pudo iniciar la cámara");
}

bool Camera::get_frame(cv::OutputArray frame) {
	return cam.read(frame);
}

// OJO: es seguro porque el cvThread empieza DESPUÉS de que se declara un Renderer en el main (que inicializa GLEW/GLFW).
// NO llamar antes!!!
void Camera::set_frame() {
    cv::Mat frame;
    if (!get_frame(frame)) return;
    
    if (shared.opencv_frames.size() > 10) {
        std::cout << "[camera-opencv] Queue full, disposing\n";
        shared.opencv_frames.pop();
    }
    std::cout << "[camera] served\n";
    shared.opencv_frames.push(frame);

    if (shared.opengl_frames.size() > 10) {
        std::cout << "[camera-opengl] Queue full, disposing\n";
        shared.opengl_frames.pop();
    }
    std::cout << "[camera] served\n";
    shared.opengl_frames.push(frame);
}