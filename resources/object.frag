// SHADER DE OBJECT FRAGMENT PARA la iluminación creo que es (sí es)
#version 330 core
out vec4 FragColor;

uniform vec3 objectColor;

void main() {
    FragColor = vec4(objectColor, 1.0);
}