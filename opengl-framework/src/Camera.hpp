#pragma once
// #include "Projection_Perspective.h"
#include "EventsCallbacks.hpp"
#include "glm/glm.hpp"

namespace gl {

namespace internal {
enum class CameraControllerState {
    Idle,
    Translating,
    Rotating,
};
}

class Camera {
public:
    explicit Camera(glm::vec3 const& position = glm::vec3{5.f, 1.f, 2.f} * 0.2f, glm::vec3 const& look_at = glm::vec3{0.f});

    auto transform_matrix() const -> glm::mat4 { return _transform_matrix; }
    auto view_matrix() const -> glm::mat4 { return glm::inverse(_transform_matrix); }
    // auto      projection_matrix(float aspect_ratio) const -> glm::mat4 { return Cool::projection_matrix(_projection, aspect_ratio); }
    // auto      inverse_projection_matrix(float aspect_ratio) const -> glm::mat4 { return glm::inverse(projection_matrix(aspect_ratio)); }
    // auto      inverse_view_projection_matrix(float aspect_ratio) const -> glm::mat4;
    // auto      view_projection_matrix(float aspect_ratio) const -> glm::mat4;
    auto right_axis() const -> glm::vec3;
    auto up_axis() const -> glm::vec3;
    auto front_axis() const -> glm::vec3;
    auto position() const -> glm::vec3;
    // float     far_plane() const { return _projection.far_plane; }
    // float&    far_plane() { return _projection.far_plane; }

    // auto projection() -> Projection_Perspective& { return _projection; }
    // auto projection() const -> const Projection_Perspective& { return _projection; }

    void set_transform_matrix(glm::mat4 const& transform_matrix) { _transform_matrix = transform_matrix; }
    void set_view_matrix(glm::mat4 const& view_matrix) { _transform_matrix = glm::inverse(view_matrix); }

    /// Translation expressed in world space
    void translate(glm::vec3 const& delta_position, bool also_translate_looked_at_point = true);
    /// Rotation expressed in world space
    void rotate_around(glm::vec3 const& center, float angle, glm::vec3 const& axis);
    void rotate(float angle, glm::vec3 const& axis);

    auto events_callbacks() -> EventsCallbacks;

private:
    glm::mat4 _transform_matrix{1.f};
    glm::vec3 _looked_at{};

    internal::CameraControllerState _state{internal::CameraControllerState::Idle};
    int                             _current_button{};
    glm::vec2                       _previous_mouse_pos{};
    // Projection_Perspective _projection{};
};

} // namespace gl