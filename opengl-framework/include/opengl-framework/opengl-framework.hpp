#pragma once
#include <string_view>
#include "../../src/Camera.hpp"
#include "../../src/EventsCallbacks.hpp"
#include "../../src/Mesh.hpp"
#include "../../src/RenderTarget.hpp"
#include "../../src/Shader.hpp"
#include "../../src/Texture.hpp"
#include "../../src/make_absolute_path.hpp"
#include "glad/gl.h"
#include "glm/glm.hpp"
#include "tiny_obj_loader.h"

namespace gl {

/// Must be the very first line of your program.
void init(std::string_view window_title);

void maximize_window();

void set_events_callbacks(std::vector<EventsCallbacks>);

/// Must only be used as the condition of a while loop: `while(gl::window_is_open()) {/*do your rendering here*/}`
[[nodiscard("You must only use this function as the condition of a while loop: `while(gl::window_is_open()) {/*do your rendering here*/}`")]] auto
    window_is_open() -> bool;

auto framebuffer_width_in_pixels() -> int;
auto framebuffer_height_in_pixels() -> int;
auto framebuffer_aspect_ratio() -> float;
auto window_width_in_screen_coordinates() -> int;
auto window_height_in_screen_coordinates() -> int;
auto window_aspect_ratio() -> float;

auto time_in_seconds() -> float;
auto delta_time_in_seconds() -> float;

auto mouse_position() -> glm::vec2;

void bind_default_shader();
auto sphere_vertices();

} // namespace gl