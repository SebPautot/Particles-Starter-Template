#pragma once
#include <functional>
#include "Texture.hpp"
#include "glad/gl.h"

namespace gl {

namespace internal {
class UniqueFramebuffer {
public:
    UniqueFramebuffer() // NOLINT(*-member-init)
    {
        glGenFramebuffers(1, &_id);
    }
    ~UniqueFramebuffer()
    {
        glDeleteFramebuffers(1, &_id);
    }
    UniqueFramebuffer(UniqueFramebuffer const&)                    = delete; // You cannot copy
    auto operator=(UniqueFramebuffer const&) -> UniqueFramebuffer& = delete; // a RenderTarget. But you can move it, using std::move(my_render_target)
    UniqueFramebuffer(UniqueFramebuffer&& o) noexcept
        : _id{o._id}
    {
        o._id = 0;
    }
    auto operator=(UniqueFramebuffer&& o) noexcept -> UniqueFramebuffer&
    {
        if (&o != this)
        {
            glDeleteFramebuffers(1, &_id);
            _id   = o._id;
            o._id = 0;
        }
        return *this;
    }

    auto id() const { return _id; }

private:
    GLuint _id;
};
} // namespace internal

struct ColorAttachment_Descriptor {
    InternalFormat_Color format{};
    TextureOptions       options{};
};

struct DepthStencilAttachment_Descriptor {
    InternalFormat_DepthStencil format{};
    TextureOptions              options{};
};

struct RenderTarget_Descriptor {
    GLsizei                                          width{};
    GLsizei                                          height{};
    std::vector<ColorAttachment_Descriptor>          color_textures{};
    std::optional<DepthStencilAttachment_Descriptor> depth_stencil_texture{};
};

class RenderTarget {
public:
    explicit RenderTarget(RenderTarget_Descriptor const&);

    void render(std::function<void()> const& render_fn);
    void resize(GLsizei width, GLsizei height);

    auto color_texture(size_t index) const -> Texture const& { return _color_textures.at(index); }
    auto depth_stencil_texture() const -> Texture const&
    {
        assert(_depth_stencil_texture.has_value() && "You didn't create this RenderTarget with a depth_stencil_texture. See RenderTarget_Descriptor.");
        return *_depth_stencil_texture;
    }

private:
    void create_attachments(RenderTarget_Descriptor const& desc);

private:
    internal::UniqueFramebuffer _id{};
    std::vector<Texture>        _color_textures{};
    std::optional<Texture>      _depth_stencil_texture{};

    RenderTarget_Descriptor _desc{};
};

} // namespace gl