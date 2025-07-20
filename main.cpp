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

/*
void drawSphere(float radius = 1.0f, int segments = 32) {
    glColor3f(0.8f, 0.8f, 0.2f);

    // Material properties for lighting
    GLfloat mat_ambient[] = { 0.7f, 0.7f, 0.2f, 1.0f };
    GLfloat mat_diffuse[] = { 0.8f, 0.8f, 0.2f, 1.0f };
    glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);

    // Fixed sphere generation (corrected divisions)
    for (int i = 0; i <= segments; ++i) {
        float lat0 = glm::pi<float>() * (-0.5f + float(i - 1) / segments);
        float z0 = radius * sin(lat0);
        float zr0 = radius * cos(lat0);

        float lat1 = glm::pi<float>() * (-0.5f + float(i) / segments);
        float z1 = radius * sin(lat1);
        float zr1 = radius * cos(lat1);

        glBegin(GL_QUAD_STRIP);
        for (int j = 0; j <= segments; ++j) {
            float lng = 2 * glm::pi<float>() * float(j) / segments;
            float x = cos(lng);
            float y = sin(lng);

            glNormal3f(x * zr0, y * zr0, z0);
            glVertex3f(x * zr0, y * zr0, z0);
            glNormal3f(x * zr1, y * zr1, z1);
            glVertex3f(x * zr1, y * zr1, z1);
        }
        glEnd();
    }
}
*/

int main() {
    SharedData shared;
    Renderer renderer(shared);

    renderer.run();

}