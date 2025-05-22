#pragma once
#include <variant>
#include <vector>
#include "glad/gl.h"

namespace gl {

namespace internal {
class VertexAttribute_Base {
public:
    explicit VertexAttribute_Base(int index)
        : _index{index}
    {}

    auto index() const -> int { return _index; }

private:
    int _index{};
};
} // namespace internal

namespace VertexAttribute {
class Float : public internal::VertexAttribute_Base {
public:
    using VertexAttribute_Base::VertexAttribute_Base;
    static auto size() -> GLint { return 1; }
    static auto type() -> GLenum { return GL_FLOAT; }
};
class Vec2 : public internal::VertexAttribute_Base {
public:
    using VertexAttribute_Base::VertexAttribute_Base;
    static auto size() -> GLint { return 2; }
    static auto type() -> GLenum { return GL_FLOAT; }
};
class Vec3 : public internal::VertexAttribute_Base {
public:
    using VertexAttribute_Base::VertexAttribute_Base;
    static auto size() -> GLint { return 3; }
    static auto type() -> GLenum { return GL_FLOAT; }
};
class Vec4 : public internal::VertexAttribute_Base {
public:
    using VertexAttribute_Base::VertexAttribute_Base;
    static auto size() -> GLint { return 4; }
    static auto type() -> GLenum { return GL_FLOAT; }
};
class Int : public internal::VertexAttribute_Base {
public:
    using VertexAttribute_Base::VertexAttribute_Base;
    static auto size() -> GLint { return 1; }
    static auto type() -> GLenum { return GL_INT; }
};
class IVec2 : public internal::VertexAttribute_Base {
public:
    using VertexAttribute_Base::VertexAttribute_Base;
    static auto size() -> GLint { return 2; }
    static auto type() -> GLenum { return GL_INT; }
};
class IVec3 : public internal::VertexAttribute_Base {
public:
    using VertexAttribute_Base::VertexAttribute_Base;
    static auto size() -> GLint { return 3; }
    static auto type() -> GLenum { return GL_INT; }
};
class IVec4 : public internal::VertexAttribute_Base {
public:
    using VertexAttribute_Base::VertexAttribute_Base;
    static auto size() -> GLint { return 4; }
    static auto type() -> GLenum { return GL_INT; }
};

using Position2D = Vec2;
using Position3D = Vec3;
using Normal3D   = Vec3;
using UV         = Vec2;
using ColorRGB   = Vec3;
using ColorRGBA  = Vec4;
} // namespace VertexAttribute

using AnyVertexAttribute = std::variant<
    VertexAttribute::Float,
    VertexAttribute::Vec2,
    VertexAttribute::Vec3,
    VertexAttribute::Vec4,
    VertexAttribute::Int,
    VertexAttribute::IVec2,
    VertexAttribute::IVec3,
    VertexAttribute::IVec4>;

struct VertexBuffer_Descriptor {
    std::vector<AnyVertexAttribute> const& layout; // NOLINT(*avoid-const-or-ref-data-members)
    std::vector<float> const&              data;   // NOLINT(*avoid-const-or-ref-data-members)
};

struct Mesh_Descriptor {
    std::vector<VertexBuffer_Descriptor> const& vertex_buffers; // NOLINT(*avoid-const-or-ref-data-members)
    std::vector<uint32_t> const&                index_buffer{};
};

class Mesh {
public:
    explicit Mesh(Mesh_Descriptor);
    ~Mesh();
    Mesh(Mesh const&)                    = delete; // You cannot copy
    auto operator=(Mesh const&) -> Mesh& = delete; // a Mesh. But you can move it, using std::move(my_mesh)
    Mesh(Mesh&&) noexcept;
    auto operator=(Mesh&&) noexcept -> Mesh&;

    void draw() const;

private:
    GLuint              _vertex_array{};
    std::vector<GLuint> _vertex_buffers{};
    GLuint              _maybe_index_buffer{};

    size_t _triangles_count{};
};

} // namespace gl