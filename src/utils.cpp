#include "utils.hpp"
#include <random>
#include "opengl-framework/opengl-framework.hpp"

namespace utils {

static auto& generator()
{
    thread_local std::default_random_engine gen{std::random_device{}()};
    return gen;
}

float rand(float min, float max)
{
    return std::uniform_real_distribution<float>{min, max}(generator());
}

static auto make_square_mesh() -> gl::Mesh
{
    return gl::Mesh{gl::Mesh_Descriptor{
        .vertex_buffers = {
            gl::VertexBuffer_Descriptor{
                .layout = {gl::VertexAttribute::Position2D(0), gl::VertexAttribute::UV(1)},
                .data   = {
                    -1.f, -1.f, 0.f, 0.f, //
                    +1.f, -1.f, 1.f, 0.f, //
                    +1.f, +1.f, 1.f, 1.f, //
                    -1.f, +1.f, 0.f, 1.f  //
                }
            }
        },
        .index_buffer = {0, 1, 2, 0, 2, 3},
    }};
}

static auto make_disk_shader() -> gl::Shader
{
    return gl::Shader{
        gl::Shader_Descriptor{
            .vertex   = gl::ShaderSource::Code({R"GLSL(
#version 410

layout(location = 0) in vec2 in_position;
layout(location = 1) in vec2 in_uv;

uniform vec2 u_position;
uniform float u_radius;
uniform float u_inverse_aspect_ratio;

out vec2 v_uv;

void main()
{
    vec2 position = u_position + u_radius * in_position;

    gl_Position = vec4(position * vec2(u_inverse_aspect_ratio, 1.), 0., 1.);
    v_uv = in_uv;
}
)GLSL"}),
            .fragment = gl::ShaderSource::Code({R"GLSL(
#version 410

out vec4 out_color;

in vec2 v_uv;
uniform vec4 u_color;

void main()
{
    vec2 dir = v_uv - vec2(0.5);
    if (dot(dir, dir) > 0.25)
        discard;
    out_color = u_color;
}
)GLSL"}),
        }
    };
}

void draw_disk(glm::vec2 position, float radius, glm::vec4 const& color)
{
    static auto square_mesh = make_square_mesh();
    static auto disk_shader = make_disk_shader();

    disk_shader.bind();
    disk_shader.set_uniform("u_position", position);
    disk_shader.set_uniform("u_radius", radius);
    disk_shader.set_uniform("u_inverse_aspect_ratio", 1.f / gl::framebuffer_aspect_ratio());
    disk_shader.set_uniform("u_color", color);
    square_mesh.draw();
}

} // namespace utils