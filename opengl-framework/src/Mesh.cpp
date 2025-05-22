#include "Mesh.hpp"
#include <cassert>
#include <numeric>
#include <opengl-framework/opengl-framework.hpp>

namespace gl {

static auto index(AnyVertexAttribute const& attr)
{
    return std::visit([](auto&& attr) { return attr.index(); }, attr);
}
static auto size(AnyVertexAttribute const& attr)
{
    return std::visit([](auto&& attr) { return attr.size(); }, attr);
}
static auto type(AnyVertexAttribute const& attr)
{
    return std::visit([](auto&& attr) { return attr.type(); }, attr);
}
static auto size_in_bytes(AnyVertexAttribute const& attr)
{
    return size(attr) * 4;
}

Mesh::Mesh(Mesh_Descriptor desc)
{
    assert(!desc.vertex_buffers.empty() && "You must provide at least one vertex buffer to construct a mesh.");

    if (!desc.index_buffer.empty())
    {
        assert(desc.index_buffer.size() % 3 == 0 && "You must provide 3 indices for each triangle");
        _triangles_count = desc.index_buffer.size() / 3;
    }

    { // Vertex Array
        glGenVertexArrays(1, &_vertex_array);
        glBindVertexArray(_vertex_array);
    }

    { // Vertex Buffers
        _vertex_buffers.resize(desc.vertex_buffers.size());
        glGenBuffers(static_cast<int>(_vertex_buffers.size()), _vertex_buffers.data());
        for (size_t i = 0; i < _vertex_buffers.size(); ++i)
        {
            glBindBuffer(GL_ARRAY_BUFFER, _vertex_buffers[i]);
            glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(desc.vertex_buffers[i].data.size() * sizeof(GLfloat)), desc.vertex_buffers[i].data.data(), GL_STATIC_DRAW);

            int const stride = std::accumulate(desc.vertex_buffers[i].layout.begin(), desc.vertex_buffers[i].layout.end(), 0, [](int acc, AnyVertexAttribute const& attr) {
                return acc + size_in_bytes(attr);
            });
            if (desc.index_buffer.empty())
            {
                auto const triangles_count = desc.vertex_buffers[i].data.size() / (stride / sizeof(float)) / 3;
                if (i == 0)
                    _triangles_count = triangles_count;
                else
                    assert(_triangles_count == triangles_count && "Some vertex buffers contain more vertices than others! Make sure that their data is correct, and that the layout matches the data.");
            }
            uint64_t pointer{0};
            for (auto const& attribute : desc.vertex_buffers[i].layout)
            {
                glEnableVertexAttribArray(index(attribute));
                glVertexAttribPointer(index(attribute), size(attribute), type(attribute), GL_FALSE, stride, reinterpret_cast<void*>(pointer)); // NOLINT(*reinterpret-cast, performance-no-int-to-ptr)
                pointer += size_in_bytes(attribute);
            }
        }
    }

    { // Index Buffer
        if (!desc.index_buffer.empty())
        {
            glGenBuffers(1, &_maybe_index_buffer);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _maybe_index_buffer);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(desc.index_buffer.size() * sizeof(uint32_t)), desc.index_buffer.data(), GL_STATIC_DRAW);
        }
    }
}

void Mesh::draw() const
{
    glBindVertexArray(_vertex_array);
    if (_maybe_index_buffer != 0)
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(3 * _triangles_count), GL_UNSIGNED_INT, reinterpret_cast<void*>(0)); // NOLINT(*reinterpret-cast)
    else
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(3 * _triangles_count));
}

Mesh::~Mesh()
{
    glDeleteVertexArrays(1, &_vertex_array);
    if (!_vertex_buffers.empty()) // Might have been moved-from
        glDeleteBuffers(static_cast<int>(_vertex_buffers.size()), _vertex_buffers.data());
    glDeleteBuffers(1, &_maybe_index_buffer);
}

Mesh::Mesh(Mesh&& o) noexcept
    : _vertex_array{o._vertex_array}
    , _vertex_buffers{std::move(o._vertex_buffers)}
    , _maybe_index_buffer{o._maybe_index_buffer}
    , _triangles_count{o._triangles_count}
{
    o._vertex_array = 0;
    o._vertex_buffers.resize(0);
    o._maybe_index_buffer = 0;
}

auto Mesh::operator=(Mesh&& o) noexcept -> Mesh&
{
    if (this != &o)
    {
        // Delete this
        glDeleteVertexArrays(1, &_vertex_array);
        if (!_vertex_buffers.empty()) // Might have been moved-from
            glDeleteBuffers(static_cast<int>(_vertex_buffers.size()), _vertex_buffers.data());
        glDeleteBuffers(1, &_maybe_index_buffer);

        // Move
        _vertex_array       = o._vertex_array;
        _vertex_buffers     = std::move(o._vertex_buffers);
        _maybe_index_buffer = o._maybe_index_buffer;
        _triangles_count    = o._triangles_count;

        o._vertex_array = 0;
        o._vertex_buffers.resize(0);
        o._maybe_index_buffer = 0;
    }
    return *this;
}

} // namespace gl