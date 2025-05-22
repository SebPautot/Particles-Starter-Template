#include "Texture.hpp"
#include <cassert>
#include "glm/gtc/type_ptr.hpp"
#include "img/img.hpp"
#include "make_absolute_path.hpp"

namespace gl {

static void upload_image_data(TextureSource::Pixels const& source)
{
    glTexImage2D(GL_TEXTURE_2D, 0, static_cast<GLint>(source.texture_format), source.width, source.height, 0, static_cast<GLenum>(source.source_pixels_format), static_cast<GLenum>(source.source_pixels_type), source.pixels.data());
}

static void upload_image_data(TextureSource::EmptyImage const& source)
{
    glTexStorage2D(GL_TEXTURE_2D, 1, static_cast<GLint>(source.texture_format), source.width, source.height);
}

static void upload_image_data(TextureSource::File const& source)
{
    auto const image = img::load(make_absolute_path(source.path), 4, source.flip_y);
    upload_image_data(TextureSource::Pixels{.pixels = image.data_span(), .width = static_cast<GLsizei>(image.width()), .height = static_cast<GLsizei>(image.height()), .source_pixels_type = Type::UnsignedByte, .source_pixels_format = Format::RGBA, .texture_format = source.texture_format});
}

Texture::Texture(AnyTextureSource const& source, TextureOptions const& options)
{
    glBindTexture(GL_TEXTURE_2D, _id.id());
    std::visit([&](auto&& source) { upload_image_data(source); }, source);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, static_cast<GLint>(options.minification_filter));
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, static_cast<GLint>(options.magnification_filter));
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, static_cast<GLint>(options.wrap_x));
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, static_cast<GLint>(options.wrap_y));
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, glm::value_ptr(options.border_color));
}

} // namespace gl