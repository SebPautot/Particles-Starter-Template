#version 410
layout(location = 0) in vec3 in_position;

uniform mat4 _view_projection_matrix;
out vec3     _position;

void main()
{
    gl_Position = _view_projection_matrix * vec4(in_position, 1.);
    _position   = in_position;
}