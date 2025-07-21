#include "shaders.h"

Shader::Shader(const std::string& vert_path, const std::string& frag_path) {
	GLuint vert_shader = shader_compiler(GL_VERTEX_SHADER, vert_path);
	GLuint frag_shader = shader_compiler(GL_FRAGMENT_SHADER, frag_path);

	program = glCreateProgram();
	glAttachShader(program, vert_shader);
	glAttachShader(program, frag_shader);
	glLinkProgram(program);

	GLint success;
	glGetProgramiv(program, GL_LINK_STATUS, &success);
	if (!success) {
		std::string info_log;
		glGetProgramInfoLog(program, 512, nullptr, info_log.data());
		std::cerr << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << info_log << std::endl;
	}

	glDeleteShader(vert_shader);
	glDeleteShader(frag_shader);
}

Shader::~Shader() {
	if (program != 0)
		glDeleteProgram(program);
}

void Shader::use() const {
	glUseProgram(program);
}

GLuint Shader::shader_compiler(GLenum type, const std::string& path) const {
	std::string src = load_source(path);
	const char* c_src = src.c_str();

	GLuint shader = glCreateShader(type);
	glShaderSource(shader, 1, &c_src, nullptr);
	glCompileShader(shader);

	GLint success;
	glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
	if (!success) {
		std::string info_log;
		glGetProgramInfoLog(program, 512, nullptr, info_log.data());
		std::cerr << "ERROR::SHADER::COMPILATION_FAILED\n" << info_log << std::endl;
	}

	return shader;
}

std::string Shader::load_source(const std::string& path) const {
	std::ifstream file(path);
	if (!file.is_open()) {
		std::cerr << "ERROR::SHADER::FILE_NOT_FOUND " << path << std::endl;
		return "";
	}

	std::stringstream buffer;
	buffer << file.rdbuf();
	return buffer.str();
}
