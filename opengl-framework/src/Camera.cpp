#include "Camera.hpp"
#include "EventsCallbacks.hpp"
#include "glfw.hpp"
#include "glm/common.hpp"
#include "glm/ext/matrix_transform.hpp"
#include "glm/gtc/matrix_access.hpp"

namespace gl {

Camera::Camera(glm::vec3 const& position, glm::vec3 const& look_at)
    : _transform_matrix{glm::inverse(glm::lookAt(
          position,
          look_at,
          glm::vec3(0.f, 0.f, 1.f)
      ))}
    , _looked_at{look_at}
{
}

auto Camera::right_axis() const -> glm::vec3
{
    return glm::normalize(glm::column(_transform_matrix, 0));
}
auto Camera::up_axis() const -> glm::vec3
{
    return glm::normalize(glm::column(_transform_matrix, 1));
}
auto Camera::front_axis() const -> glm::vec3
{
    return -glm::normalize(glm::column(_transform_matrix, 2));
}
auto Camera::position() const -> glm::vec3
{
    return glm::column(_transform_matrix, 3);
}

void Camera::translate(glm::vec3 const& delta_position, bool also_translate_looked_at_point)
{
    _transform_matrix = glm::translate(glm::mat4{1.f}, delta_position) * _transform_matrix;
    if (also_translate_looked_at_point)
        _looked_at += delta_position;
}

void Camera::rotate_around(glm::vec3 const& center, float angle, glm::vec3 const& axis)
{
    translate(-center, false);
    rotate(angle, axis);
    translate(+center, false);
}

void Camera::rotate(float angle, glm::vec3 const& axis)
{
    _transform_matrix = glm::rotate(glm::mat4{1.f}, angle, axis) * _transform_matrix;
}

auto Camera::events_callbacks() -> EventsCallbacks
{
    return EventsCallbacks{
        .on_mouse_moved = [&](MouseMoveEvent const& e) {
            auto const delta    = _previous_mouse_pos - e.position;
            _previous_mouse_pos = e.position;
            if (_state == internal::CameraControllerState::Rotating)
            {
                rotate_around(_looked_at, delta.x * 0.003f * glm::sign(up_axis().z), glm::vec3{0.f, 0.f, 1.f});
                rotate_around(_looked_at, delta.y * 0.003f, right_axis());
            }
            else if (_state == internal::CameraControllerState::Translating)
            {
                translate((delta.x * right_axis() - delta.y * up_axis()) * 0.003f);
            }
            //
        },
        .on_mouse_pressed = [&](MousePressedEvent const& e) {
            if (_state != internal::CameraControllerState::Idle)
                return;
            _current_button     = e.button;
            _previous_mouse_pos = e.position;
            if (e.button == GLFW_MOUSE_BUTTON_LEFT && !(e.mods & GLFW_MOD_SHIFT))
                _state = internal::CameraControllerState::Rotating;
            else
                _state = internal::CameraControllerState::Translating;
            //
        },
        .on_mouse_released = [&](MouseReleasedEvent const& e) {
            if (e.button == _current_button)
                _state = internal::CameraControllerState::Idle;
            //
        },
        .on_scroll = [&](ScrollEvent const& e) {
            if (_state != internal::CameraControllerState::Idle)
                return;
            glm::vec3 const prev_dir = position() - _looked_at;
            auto const      new_dir  = prev_dir * std::pow(0.93f, e.scroll);
            translate(new_dir - prev_dir, false /*also_translate_looked_at_point*/);
            //
        },
    };
}

} // namespace gl