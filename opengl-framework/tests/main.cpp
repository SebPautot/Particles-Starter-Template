#include "glm/gtc/matrix_transform.hpp"
#include "opengl-framework/opengl-framework.hpp"

int main()
{
    gl::init("OpenGL Framework");
    gl::maximize_window();
    glEnable(GL_DEPTH_TEST);

    auto camera = gl::Camera{};

    gl::set_events_callbacks({
        camera.events_callbacks(),
    });

    // TODO talk about default coordinates, and that they stretch with window : we will fix this by writing our own shader
    auto const triangle_mesh = gl::Mesh{{
        .vertex_buffers = {{
            .layout = {gl::VertexAttribute::Position3D{0 /*must match the layout index in the shader*/}},
            .data   = {
                -1, -1, -1,
                -1, +1, -1,
                +1, +1, -1,
                +1, -1, -1,

                -1, -1, +1,
                -1, +1, +1,
                +1, +1, +1,
                +1, -1, +1
                // clang-format on
            },
        }},
        .index_buffer   = {
            // clang-format off
            0, 1, 2,
            0, 2, 3,
            
            1, 5, 6,
            1, 2, 6,

            2, 6, 7,
            2, 3, 7,

            3, 7, 4,
            3, 0, 4,

            0, 4, 5,
            0, 1, 5,
            
            4, 5, 6,
            4, 6, 7
            // clang-format on
        },
    }};

    auto const shader = gl::Shader{{
        .vertex   = gl::ShaderSource::File{"res/vertex.glsl"},
        .fragment = gl::ShaderSource::File{"res/fragment.glsl"},
    }};

    while (gl::window_is_open())
    {
        glClearColor(0.f, 0.f, 1.f, 1.f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        shader.bind();
        shader.set_uniform("_view_projection_matrix", glm::infinitePerspective(1.f, gl::window_aspect_ratio(), 0.001f) * camera.view_matrix());
        triangle_mesh.draw();
        // camera.rotate(0.01f, {0.f, 0.f, 1.f});
        // Ensuite dessinez un carré
        // Puis allez voir dans RenderDoc son wireframe etc
        // Puis on va loader un obj (avant, faire un cube à la mano? Mais faut un minimum de lighting 3D pour qu'il rende bien, et une caméra pour qu'on puisse bouger et comprendre ce qu'il se passe)
    }
}