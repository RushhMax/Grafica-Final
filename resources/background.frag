// SHADER DE BACKGROUND FRAGMENT PARA colorear
#version 330 core
in vec2 UV;
out vec4 color;

uniform sampler2D backgroundTexture;

void main(){
    color = texture(backgroundTexture, UV);
}