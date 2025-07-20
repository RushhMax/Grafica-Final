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
	std::lock_guard lock(shared.mut);
	if (!get_frame(shared.frame))
		CV_Error(cv::Error::StsObjectNotFound, "No se pudo leer el frame");
	else {
        CV_Assert(!shared.frame.empty() && shared.frame.type() == CV_8UC3);

        glGenTextures(1, &shared.cam_texture);
        glBindTexture(GL_TEXTURE_2D, shared.cam_texture);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT);

        glTexImage2D(
            GL_TEXTURE_2D, 0, GL_RGB,
            shared.frame.cols, shared.frame.rows, 0,
            GL_BGR, GL_UNSIGNED_BYTE, shared.frame.ptr()
        );

        glGenerateMipmap(GL_TEXTURE_2D);
	}
}