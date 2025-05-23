#include "../include/opengl-framework/opengl-framework.hpp"
#include <glad/gl.h>
#include <cassert>
#include <format>
#include <iostream>
#include <vector>
#include "Camera.hpp"
#include "GLFW/glfw3.h"
#include "Shader.hpp"
#include "glfw.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "handle_error.hpp"

namespace {
struct Context { // NOLINT(*special-member-functions)
    GLFWwindow*                      window{nullptr};
    std::vector<gl::EventsCallbacks> events_callbacks{};
    float                            last_time{0.f};
    float                            delta_time{0.f};
    bool                             is_first_frame{true};

    ~Context()
    {
        glfwDestroyWindow(window);
    }
};

auto context() -> Context&
{
    static auto instance = Context{};
    return instance;
}

void APIENTRY opengl_debug_callback(
    GLenum       source, // NOLINT(bugprone-easily-swappable-parameters)
    GLenum       type,
    unsigned int id,
    GLenum       severity,
    GLsizei /*length*/,
    const char* base_message,
    const void* /*userParam*/
)
{
    // Ignore non-significant error / warning codes
    if (id == 131169 || id == 131185 || id == 131218 || id == 131204)
        return;

    { // Check if we have already seen the message
        static auto already_seen_ids = std::vector<unsigned int>{};
        if (std::find(already_seen_ids.begin(), already_seen_ids.end(), id) != already_seen_ids.end())
            return;
        already_seen_ids.push_back(id);
    }

    // Make message
    auto message = std::string{};
    message += std::format("[OpenGL] Debug message (id={})\n{}", id, base_message);

    switch (source)
    {
    case GL_DEBUG_SOURCE_API: message += "\n\nSource: API"; break;
    case GL_DEBUG_SOURCE_WINDOW_SYSTEM: message += "\n\nSource: Window System"; break;
    case GL_DEBUG_SOURCE_SHADER_COMPILER: message += "\n\nSource: Shader Compiler"; break;
    case GL_DEBUG_SOURCE_THIRD_PARTY: message += "\n\nSource: Third Party"; break;
    case GL_DEBUG_SOURCE_APPLICATION: message += "\n\nSource: Application"; break;
    case GL_DEBUG_SOURCE_OTHER: message += "\n\nSource: Other"; break;
    default: message += "\n\nSource: Unknown"; break;
    }

    switch (type)
    {
    case GL_DEBUG_TYPE_ERROR: message += "\nType: Error"; break;
    case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR: message += "\nType: Deprecated Behaviour"; break;
    case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR: message += "\nType: Undefined Behaviour"; break;
    case GL_DEBUG_TYPE_PORTABILITY: message += "\nType: Portability"; break;
    case GL_DEBUG_TYPE_PERFORMANCE: message += "\nType: Performance"; break;
    case GL_DEBUG_TYPE_MARKER: message += "\nType: Marker"; break;
    case GL_DEBUG_TYPE_PUSH_GROUP: message += "\nType: Push Group"; break;
    case GL_DEBUG_TYPE_POP_GROUP: message += "\nType: Pop Group"; break;
    case GL_DEBUG_TYPE_OTHER: message += "\nType: Other"; break;
    default: message += "\nType: Unknown"; break;
    }

    switch (severity)
    {
    case GL_DEBUG_SEVERITY_HIGH: message += "\nSeverity: High"; break;
    case GL_DEBUG_SEVERITY_MEDIUM: message += "\nSeverity: Medium"; break;
    case GL_DEBUG_SEVERITY_LOW: message += "\nSeverity: Low"; break;
    case GL_DEBUG_SEVERITY_NOTIFICATION: message += "\nSeverity: Notification"; break;
    default: message += "\nSeverity: Unknown"; break;
    }

    // Log
    if (type == GL_DEBUG_TYPE_ERROR || type == GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR || severity == GL_DEBUG_SEVERITY_HIGH)
        gl::handle_error(message);
    else
        std::cout << message << '\n';
}

void assert_init_has_been_called()
{
    assert(context().window != nullptr && "You must call gl::init() as the first line of your program.");
}

void mouse_move_callback(GLFWwindow*, double x_pos, double y_pos)
{
    for (auto const& callbacks : context().events_callbacks)
        callbacks.on_mouse_moved({.position = glm::vec2{static_cast<float>(x_pos), static_cast<float>(y_pos)}});
}
void mouse_button_callback(GLFWwindow*, int button, int action, int mods)
{
    double x, y;
    glfwGetCursorPos(context().window, &x, &y);
    if (action == GLFW_PRESS)
    {
        for (auto const& callbacks : context().events_callbacks)
            callbacks.on_mouse_pressed({.position = glm::vec2{static_cast<float>(x), static_cast<float>(y)}, .button = button, .mods = mods});
    }
    else
    {
        assert(action == GLFW_RELEASE);
        for (auto const& callbacks : context().events_callbacks)
            callbacks.on_mouse_released({.position = glm::vec2{static_cast<float>(x), static_cast<float>(y)}, .button = button, .mods = mods});
    }
}
void scroll_callback(GLFWwindow*, double x_offset, double y_offset)
{
    for (auto const& callbacks : context().events_callbacks)
        callbacks.on_scroll({.scroll = static_cast<float>(y_offset), .horizontal_scroll = static_cast<float>(x_offset)});
}
void framebuffer_resized_callback(GLFWwindow*, int width_in_pixels, int height_in_pixels)
{
    glViewport(0, 0, width_in_pixels, height_in_pixels);
    for (auto const& callbacks : context().events_callbacks)
        callbacks.on_framebuffer_resized({.width_in_pixels = width_in_pixels, .height_in_pixels = height_in_pixels});
}
void window_resized_callback(GLFWwindow*, int width_in_screen_coordinates, int height_in_screen_coordinates)
{
    for (auto const& callbacks : context().events_callbacks)
        callbacks.on_window_resized({.width_in_screen_coordinates = width_in_screen_coordinates, .height_in_screen_coordinates = height_in_screen_coordinates});
}

} // namespace

namespace gl {

void init(std::string_view window_title)
{
    assert(context().window == nullptr && "You are calling gl::init() twice. You must only call it once.");

    glfwSetErrorCallback([](int, const char* error_message) {
        handle_error(std::format("[glfw error] {}", error_message));
    });
    if (!glfwInit())
        handle_error("[opengl_framework] Failed to initialize glfw");
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
#if !defined(__APPLE__)
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3); // OpenGL 4.3 allows us to use improved debugging. But it is not available on MacOS.
#else
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
#endif
#if !defined(NDEBUG) && !defined(__APPLE__)
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, true);
#endif
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // Required on MacOS
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);           // Required on MacOS
    context().window = glfwCreateWindow(1280, 720, window_title.data(), nullptr, nullptr);
    if (!context().window)
        handle_error("[opengl_framework] Failed to create the window");
    glfwMakeContextCurrent(context().window);
    if (!gladLoadGL(glfwGetProcAddress))
        handle_error("[opengl_framework] Failed to initialize glad");

#if !defined(NDEBUG) && !defined(__APPLE__)
    int flags; // NOLINT(*init-variables)
    glGetIntegerv(GL_CONTEXT_FLAGS, &flags);
    if (flags & GL_CONTEXT_FLAG_DEBUG_BIT)
    {
        glEnable(GL_DEBUG_OUTPUT);
        glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
        glDebugMessageCallback(&opengl_debug_callback, nullptr);
        glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_TRUE);
    }
    else
    {
        std::cerr << "[opengl_framework] Unable to create an OpenGL debug context\n";
    }
#endif
    glfwSetCursorPosCallback(context().window, &mouse_move_callback);
    glfwSetMouseButtonCallback(context().window, &mouse_button_callback);
    glfwSetScrollCallback(context().window, &scroll_callback);
    glfwSetWindowSizeCallback(context().window, &window_resized_callback);
    glfwSetFramebufferSizeCallback(context().window, &framebuffer_resized_callback);
}

void maximize_window()
{
    assert_init_has_been_called();
    glfwMaximizeWindow(context().window);
}

void set_events_callbacks(std::vector<EventsCallbacks> callbacks)
{
    context().events_callbacks = std::move(callbacks);
}

auto window_is_open() -> bool
{
    assert_init_has_been_called();

    float const time = time_in_seconds();
    if (!context().is_first_frame)
        context().delta_time = time - context().last_time;
    context().last_time = time;

    glfwSwapBuffers(context().window);
    glfwPollEvents();
    context().is_first_frame = false;
    return !glfwWindowShouldClose(context().window);
}

auto framebuffer_width_in_pixels() -> int
{
    int w; // NOLINT(*init-variables)
    glfwGetFramebufferSize(context().window, &w, nullptr);
    return w;
}

auto framebuffer_height_in_pixels() -> int
{
    int h; // NOLINT(*init-variables)
    glfwGetFramebufferSize(context().window, nullptr, &h);
    return h;
}

auto framebuffer_aspect_ratio() -> float
{
    int w, h; // NOLINT(*init-variables)
    glfwGetFramebufferSize(context().window, &w, &h);
    return static_cast<float>(w) / static_cast<float>(h);
}

auto window_width_in_screen_coordinates() -> int
{
    int w; // NOLINT(*init-variables)
    glfwGetWindowSize(context().window, &w, nullptr);
    return w;
}

auto window_height_in_screen_coordinates() -> int
{
    int h; // NOLINT(*init-variables)
    glfwGetWindowSize(context().window, nullptr, &h);
    return h;
}

auto window_aspect_ratio() -> float
{
    int w, h; // NOLINT(*init-variables)
    glfwGetWindowSize(context().window, &w, &h);
    return static_cast<float>(w) / static_cast<float>(h);
}

auto time_in_seconds() -> float
{
    return static_cast<float>(glfwGetTime());
}

auto delta_time_in_seconds() -> float
{
    return context().delta_time;
}

auto mouse_position() -> glm::vec2
{
    double x{};
    double y{};
    glfwGetCursorPos(context().window, &x, &y);
    return glm::vec2{
        (static_cast<float>(x) - static_cast<float>(gl::window_width_in_screen_coordinates()) / 2.f) / static_cast<float>(gl::window_height_in_screen_coordinates()) * 2.f,
        static_cast<float>(y) / static_cast<float>(gl::window_height_in_screen_coordinates()) * -2.f + 1.f,
    };
}

static auto default_shader() -> Shader&
{
    static auto instance = Shader{{
        .vertex   = ShaderSource::Code{R"GLSL(
#version 410
layout(location = 0) in vec3 in_position;

void main()
{
    gl_Position = vec4(in_position, 1.);
}
)GLSL"},
        .fragment = ShaderSource::Code{R"GLSL(
#version 410
out vec4 out_color;

void main()
{
    out_color = vec4(1.);
}
)GLSL"},
    }};
    return instance;
}

void bind_default_shader()
{
    default_shader().bind();
}

} // namespace gl