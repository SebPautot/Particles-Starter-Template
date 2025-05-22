#include "RenderTarget.hpp"
#include <array>
#include "Texture.hpp"
#include "handle_error.hpp"

namespace gl {

static auto attachment_type(InternalFormat_DepthStencil format) -> GLenum
{
    switch (format)
    {
    case InternalFormat_DepthStencil::Depth16:
    case InternalFormat_DepthStencil::Depth24:
    case InternalFormat_DepthStencil::Depth32F:
        return GL_DEPTH_ATTACHMENT;
    case InternalFormat_DepthStencil::Stencil8:
        return GL_STENCIL_ATTACHMENT;
    case InternalFormat_DepthStencil::Depth24_Stencil8:
    case InternalFormat_DepthStencil::Depth32F_Stencil8:
        return GL_DEPTH_STENCIL_ATTACHMENT;
    }
}

void RenderTarget::create_attachments(RenderTarget_Descriptor const& desc)
{
    _color_textures.clear();
    render([&]() { // HACK, we reuse render() as a way to have our framebuffer bound
        if (desc.color_textures.empty())
        { // We need to explicitly do this when have no color texture
            glDrawBuffer(GL_NONE);
            glReadBuffer(GL_NONE);
        }
        for (size_t i = 0; i < desc.color_textures.size(); ++i)
        {
            auto const& color_texture = desc.color_textures[i];
            _color_textures.emplace_back(
                TextureSource::EmptyImage{
                    .width          = desc.width,
                    .height         = desc.height,
                    .texture_format = static_cast<InternalFormatSized>(color_texture.format),
                },
                color_texture.options
            );
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + i, GL_TEXTURE_2D, _color_textures.back().id(), 0);
        }
        if (desc.depth_stencil_texture.has_value())
        {
            _depth_stencil_texture.emplace(
                TextureSource::EmptyImage{
                    .width          = desc.width,
                    .height         = desc.height,
                    .texture_format = static_cast<InternalFormatSized>(desc.depth_stencil_texture->format),
                },
                desc.depth_stencil_texture->options
            );
            glFramebufferTexture2D(GL_FRAMEBUFFER, attachment_type(desc.depth_stencil_texture->format), GL_TEXTURE_2D, _depth_stencil_texture->id(), 0);
        }
        auto const status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
        if (status != GL_FRAMEBUFFER_COMPLETE)
        {
            const char* status_message = [&]() {
                switch (status)
                {
                case GL_FRAMEBUFFER_UNDEFINED:
                    return "FRAMEBUFFER_UNDEFINED";
                case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
                    return "FRAMEBUFFER_INCOMPLETE_ATTACHMENT";
                case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
                    return "FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT";
                case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
                    return "FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER";
                case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
                    return "FRAMEBUFFER_INCOMPLETE_READ_BUFFER";
                case GL_FRAMEBUFFER_UNSUPPORTED:
                    return "FRAMEBUFFER_UNSUPPORTED";
                case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE:
                    return "FRAMEBUFFER_INCOMPLETE_MULTISAMPLE";
                case GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS:
                    return "FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS";
                default:
                    return "UNKNOWN_ERROR";
                }
            }();
            handle_error(std::format("Invalid framebuffer: {}", status_message));
        }
        glClearColor(0.f, 0.f, 0.f, 0.f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Make sure to init the values in the framebuffer
    });
}

RenderTarget::RenderTarget(RenderTarget_Descriptor const& desc)
    : _desc{desc}
{
    assert(!desc.color_textures.empty() || desc.depth_stencil_texture.has_value());
    create_attachments(desc);
}

void RenderTarget::render(std::function<void()> const& render_fn)
{
    // Store previous state to restore it at the end
    int                previous_draw_framebuffer{};
    int                previous_read_framebuffer{};
    std::array<int, 4> previous_viewport{};
    glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &previous_draw_framebuffer);
    glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &previous_read_framebuffer);
    glGetIntegerv(GL_VIEWPORT, previous_viewport.data());

    // Bind our framebuffer
    glBindFramebuffer(GL_FRAMEBUFFER, _id.id());
    glViewport(0, 0, _desc.width, _desc.height);

    // Render
    render_fn();

    // Re-bind previous framebuffer
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, previous_draw_framebuffer);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, previous_read_framebuffer);
    glViewport(previous_viewport[0], previous_viewport[1], previous_viewport[2], previous_viewport[3]);
}

void RenderTarget::resize(int width, int height)
{
    _desc.width  = width;
    _desc.height = height;
    create_attachments(_desc);
}

} // namespace gl