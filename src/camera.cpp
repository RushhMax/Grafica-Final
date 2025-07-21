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
    cv::Mat temp_frame;
    if (!get_frame(temp_frame)) return;

    std::unique_lock lock(shared.mut);
    shared.cv.wait(lock, [this] { return shared.processed || !shared.running; });

    if (!shared.running) return;

    int write_idx = 1 - shared.read_idx;
    temp_frame.copyTo(shared.frames[write_idx]);
    shared.frame_ready = true;
    shared.processed = false;
    lock.unlock();
    shared.cv.notify_all();
}