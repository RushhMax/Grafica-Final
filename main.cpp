#include <numbers>
#include <stdexcept>
#include <string>

#include "renderer.h"

struct Camera {
	glm::vec3 camera_pos = { 0.0f, 0.0f, 3.0f };
    glm::vec3 camera_front = { 0.0f, 0.0f, -1.0f };
    glm::vec3 camera_up = { 0.0f, 1.0f, 0.0f };

	Camera() = default;
};

int main() {
    SharedData shared;
    Renderer renderer(shared);

    renderer.run();

}