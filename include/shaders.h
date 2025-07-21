#pragma once

#include <fstream>
#include <GL/glew.h>
#include <iostream>
#include <sstream>
#include <string>

class Shader {
	GLuint program;

	GLuint shader_compiler(GLenum, const std::string&) const;
	std::string load_source(const std::string&) const;
public:
	Shader() = default;
	Shader(const std::string&, const std::string&);
	~Shader();

	void use() const;
	GLuint id() const { return program; }
	void set_int(const std::string&, int) const;
};