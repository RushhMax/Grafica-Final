// SHADER DE OBJECT VERTEX PARA pos del objeto en opengl
#version 330 core
layout (location = 0) in vec3 aPosition;

uniform mat4 model;
uniform mat4 projection;

void main() {
    gl_Position = projection * model * vec4(aPosition, 1.0);
}