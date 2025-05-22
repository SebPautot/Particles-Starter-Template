#pragma once
#include <functional>
#include "glm/glm.hpp"

namespace gl {

struct MouseMoveEvent {
    glm::vec2 position;
};
struct MousePressedEvent {
    glm::vec2 position;
    int       button;
    int       mods;
};
struct MouseReleasedEvent {
    glm::vec2 position;
    int       button;
    int       mods;
};
struct ScrollEvent {
    float scroll;
    float horizontal_scroll;
};
// TODO documentation explaining the difference between window size and framebuffer size
struct FramebufferResizedEvent {
    int width_in_pixels{};
    int height_in_pixels{};
};
struct WindowResizedEvent {
    int width_in_screen_coordinates{};
    int height_in_screen_coordinates{};
};

struct EventsCallbacks {
    std::function<void(MouseMoveEvent const&)>          on_mouse_moved{[](auto&&) {
    }};
    std::function<void(MousePressedEvent const&)>       on_mouse_pressed{[](auto&&) {
    }};
    std::function<void(MouseReleasedEvent const&)>      on_mouse_released{[](auto&&) {
    }};
    std::function<void(ScrollEvent const&)>             on_scroll{[](auto&&) {
    }};
    std::function<void(FramebufferResizedEvent const&)> on_framebuffer_resized{[](auto&&) {
    }};
    std::function<void(WindowResizedEvent const&)>      on_window_resized{[](auto&&) {
    }};
};

} // namespace gl