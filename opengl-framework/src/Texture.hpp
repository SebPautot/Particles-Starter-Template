#pragma once
#include <filesystem>
#include <span>
#include <variant>
#include "glad/gl.h"
#include "glm/glm.hpp"

namespace gl {

/// Format in which the pixels are stored in the texture
/// See https://registry.khronos.org/OpenGL-Refpages/gl4/html/glTexImage2D.xhtml for more details
enum class InternalFormat : GLint {
    R                                  = GL_RED,
    RG                                 = GL_RG,
    RGB                                = GL_RGB,
    RGBA                               = GL_RGBA,
    Depth                              = GL_DEPTH_COMPONENT,
    DepthStencil                       = GL_DEPTH_STENCIL,
    R8                                 = GL_R8,
    R8_SNORM                           = GL_R8_SNORM,
    R16                                = GL_R16,
    R16_SNORM                          = GL_R16_SNORM,
    RG8                                = GL_RG8,
    RG8_SNORM                          = GL_RG8_SNORM,
    RG16                               = GL_RG16,
    RG16_SNORM                         = GL_RG16_SNORM,
    R3_G3_B2                           = GL_R3_G3_B2,
    RGB4                               = GL_RGB4,
    RGB5                               = GL_RGB5,
    RGB8                               = GL_RGB8,
    RGB8_SNORM                         = GL_RGB8_SNORM,
    RGB10                              = GL_RGB10,
    RGB12                              = GL_RGB12,
    RGB16_SNORM                        = GL_RGB16_SNORM,
    RGBA2                              = GL_RGBA2,
    RGBA4                              = GL_RGBA4,
    RGB5_A1                            = GL_RGB5_A1,
    RGBA8                              = GL_RGBA8,
    RGBA8_SNORM                        = GL_RGBA8_SNORM,
    RGB10_A2                           = GL_RGB10_A2,
    RGB10_A2UI                         = GL_RGB10_A2UI,
    RGBA12                             = GL_RGBA12,
    RGBA16                             = GL_RGBA16,
    SRGB8                              = GL_SRGB8,
    SRGB8_ALPHA8                       = GL_SRGB8_ALPHA8,
    R16F                               = GL_R16F,
    RG16F                              = GL_RG16F,
    RGB16F                             = GL_RGB16F,
    RGBA16F                            = GL_RGBA16F,
    R32F                               = GL_R32F,
    RG32F                              = GL_RG32F,
    RGB32F                             = GL_RGB32F,
    RGBA32F                            = GL_RGBA32F,
    R11F_G11F_B10F                     = GL_R11F_G11F_B10F,
    RGB9_E5                            = GL_RGB9_E5,
    R8I                                = GL_R8I,
    R8UI                               = GL_R8UI,
    R16I                               = GL_R16I,
    R16UI                              = GL_R16UI,
    R32I                               = GL_R32I,
    R32UI                              = GL_R32UI,
    RG8I                               = GL_RG8I,
    RG8UI                              = GL_RG8UI,
    RG16I                              = GL_RG16I,
    RG16UI                             = GL_RG16UI,
    RG32I                              = GL_RG32I,
    RG32UI                             = GL_RG32UI,
    RGB8I                              = GL_RGB8I,
    RGB8UI                             = GL_RGB8UI,
    RGB16I                             = GL_RGB16I,
    RGB16UI                            = GL_RGB16UI,
    RGB32I                             = GL_RGB32I,
    RGB32UI                            = GL_RGB32UI,
    RGBA8I                             = GL_RGBA8I,
    RGBA8UI                            = GL_RGBA8UI,
    RGBA16I                            = GL_RGBA16I,
    RGBA16UI                           = GL_RGBA16UI,
    RGBA32I                            = GL_RGBA32I,
    RGBA32UI                           = GL_RGBA32UI,
    Compressed_R                       = GL_COMPRESSED_RED,
    Compressed_RG                      = GL_COMPRESSED_RG,
    Compressed_RGB                     = GL_COMPRESSED_RGB,
    Compressed_RGBA                    = GL_COMPRESSED_RGBA,
    Compressed_SRGB                    = GL_COMPRESSED_SRGB,
    Compressed_SRGB_ALPHA              = GL_COMPRESSED_SRGB_ALPHA,
    Compressed_RED_RGTC1               = GL_COMPRESSED_RED_RGTC1,
    Compressed_SIGNED_RED_RGTC1        = GL_COMPRESSED_SIGNED_RED_RGTC1,
    Compressed_RG_RGTC2                = GL_COMPRESSED_RG_RGTC2,
    Compressed_SIGNED_RG_RGTC2         = GL_COMPRESSED_SIGNED_RG_RGTC2,
    Compressed_RGBA_BPTC_UNORM         = GL_COMPRESSED_RGBA_BPTC_UNORM,
    Compressed_SRGB_ALPHA_BPTC_UNORM   = GL_COMPRESSED_SRGB_ALPHA_BPTC_UNORM,
    Compressed_RGB_BPTC_SIGNED_FLOAT   = GL_COMPRESSED_RGB_BPTC_SIGNED_FLOAT,
    Compressed_RGB_BPTC_UNSIGNED_FLOAT = GL_COMPRESSED_RGB_BPTC_UNSIGNED_FLOAT,
};

/// Format in which the pixels are stored in the texture
/// See https://registry.khronos.org/OpenGL-Refpages/gl4/html/glTexStorage2D.xhtml for more details
enum class InternalFormatSized : GLenum {
    R8                = GL_R8,
    R8_SNORM          = GL_R8_SNORM,
    R16               = GL_R16,
    R16_SNORM         = GL_R16_SNORM,
    RG8               = GL_RG8,
    RG8_SNORM         = GL_RG8_SNORM,
    RG16              = GL_RG16,
    RG16_SNORM        = GL_RG16_SNORM,
    R3_G3_B2          = GL_R3_G3_B2,
    RGB4              = GL_RGB4,
    RGB5              = GL_RGB5,
    RGB8              = GL_RGB8,
    RGB8_SNORM        = GL_RGB8_SNORM,
    RGB10             = GL_RGB10,
    RGB12             = GL_RGB12,
    RGB16_SNORM       = GL_RGB16_SNORM,
    RGBA2             = GL_RGBA2,
    RGBA4             = GL_RGBA4,
    RGB5_A1           = GL_RGB5_A1,
    RGBA8             = GL_RGBA8,
    RGBA8_SNORM       = GL_RGBA8_SNORM,
    RGB10_A2          = GL_RGB10_A2,
    RGB10_A2UI        = GL_RGB10_A2UI,
    RGBA12            = GL_RGBA12,
    RGBA16            = GL_RGBA16,
    SRGB8             = GL_SRGB8,
    SRGB8_ALPHA8      = GL_SRGB8_ALPHA8,
    R16F              = GL_R16F,
    RG16F             = GL_RG16F,
    RGB16F            = GL_RGB16F,
    RGBA16F           = GL_RGBA16F,
    R32F              = GL_R32F,
    RG32F             = GL_RG32F,
    RGB32F            = GL_RGB32F,
    RGBA32F           = GL_RGBA32F,
    R11F_G11F_B10F    = GL_R11F_G11F_B10F,
    RGB9_E5           = GL_RGB9_E5,
    R8I               = GL_R8I,
    R8UI              = GL_R8UI,
    R16I              = GL_R16I,
    R16UI             = GL_R16UI,
    R32I              = GL_R32I,
    R32UI             = GL_R32UI,
    RG8I              = GL_RG8I,
    RG8UI             = GL_RG8UI,
    RG16I             = GL_RG16I,
    RG16UI            = GL_RG16UI,
    RG32I             = GL_RG32I,
    RG32UI            = GL_RG32UI,
    RGB8I             = GL_RGB8I,
    RGB8UI            = GL_RGB8UI,
    RGB16I            = GL_RGB16I,
    RGB16UI           = GL_RGB16UI,
    RGB32I            = GL_RGB32I,
    RGB32UI           = GL_RGB32UI,
    RGBA8I            = GL_RGBA8I,
    RGBA8UI           = GL_RGBA8UI,
    RGBA16I           = GL_RGBA16I,
    RGBA16UI          = GL_RGBA16UI,
    RGBA32I           = GL_RGBA32I,
    RGBA32UI          = GL_RGBA32UI,
    Depth32F          = GL_DEPTH_COMPONENT32F,
    Depth24           = GL_DEPTH_COMPONENT24,
    Depth16           = GL_DEPTH_COMPONENT16,
    Depth32F_Stencil8 = GL_DEPTH32F_STENCIL8,
    Depth24_Stencil8  = GL_DEPTH24_STENCIL8,
    Stencil8          = GL_STENCIL_INDEX8,
};

/// Format in which the pixels are stored in a color attachment texture
/// See https://registry.khronos.org/OpenGL-Refpages/gl4/html/glTexStorage2D.xhtml for more details
enum class InternalFormat_Color : GLenum {
    R8             = GL_R8,
    R8_SNORM       = GL_R8_SNORM,
    R16            = GL_R16,
    R16_SNORM      = GL_R16_SNORM,
    RG8            = GL_RG8,
    RG8_SNORM      = GL_RG8_SNORM,
    RG16           = GL_RG16,
    RG16_SNORM     = GL_RG16_SNORM,
    R3_G3_B2       = GL_R3_G3_B2,
    RGB4           = GL_RGB4,
    RGB5           = GL_RGB5,
    RGB8           = GL_RGB8,
    RGB8_SNORM     = GL_RGB8_SNORM,
    RGB10          = GL_RGB10,
    RGB12          = GL_RGB12,
    RGB16_SNORM    = GL_RGB16_SNORM,
    RGBA2          = GL_RGBA2,
    RGBA4          = GL_RGBA4,
    RGB5_A1        = GL_RGB5_A1,
    RGBA8          = GL_RGBA8,
    RGBA8_SNORM    = GL_RGBA8_SNORM,
    RGB10_A2       = GL_RGB10_A2,
    RGB10_A2UI     = GL_RGB10_A2UI,
    RGBA12         = GL_RGBA12,
    RGBA16         = GL_RGBA16,
    SRGB8          = GL_SRGB8,
    SRGB8_ALPHA8   = GL_SRGB8_ALPHA8,
    R16F           = GL_R16F,
    RG16F          = GL_RG16F,
    RGB16F         = GL_RGB16F,
    RGBA16F        = GL_RGBA16F,
    R32F           = GL_R32F,
    RG32F          = GL_RG32F,
    RGB32F         = GL_RGB32F,
    RGBA32F        = GL_RGBA32F,
    R11F_G11F_B10F = GL_R11F_G11F_B10F,
    RGB9_E5        = GL_RGB9_E5,
    R8I            = GL_R8I,
    R8UI           = GL_R8UI,
    R16I           = GL_R16I,
    R16UI          = GL_R16UI,
    R32I           = GL_R32I,
    R32UI          = GL_R32UI,
    RG8I           = GL_RG8I,
    RG8UI          = GL_RG8UI,
    RG16I          = GL_RG16I,
    RG16UI         = GL_RG16UI,
    RG32I          = GL_RG32I,
    RG32UI         = GL_RG32UI,
    RGB8I          = GL_RGB8I,
    RGB8UI         = GL_RGB8UI,
    RGB16I         = GL_RGB16I,
    RGB16UI        = GL_RGB16UI,
    RGB32I         = GL_RGB32I,
    RGB32UI        = GL_RGB32UI,
    RGBA8I         = GL_RGBA8I,
    RGBA8UI        = GL_RGBA8UI,
    RGBA16I        = GL_RGBA16I,
    RGBA16UI       = GL_RGBA16UI,
    RGBA32I        = GL_RGBA32I,
    RGBA32UI       = GL_RGBA32UI,
};

/// Format in which the pixels are stored in a depth-stencil attachment texture
/// See https://registry.khronos.org/OpenGL-Refpages/gl4/html/glTexStorage2D.xhtml for more details
enum class InternalFormat_DepthStencil : GLenum {
    Depth32F          = GL_DEPTH_COMPONENT32F,
    Depth24           = GL_DEPTH_COMPONENT24,
    Depth16           = GL_DEPTH_COMPONENT16,
    Depth32F_Stencil8 = GL_DEPTH32F_STENCIL8,
    Depth24_Stencil8  = GL_DEPTH24_STENCIL8,
    Stencil8          = GL_STENCIL_INDEX8,
};

/// Format of the data used to create the texture
/// See https://registry.khronos.org/OpenGL-Refpages/gl4/html/glTexImage2D.xhtml for more details
enum class Format : GLenum {
    R              = GL_RED,
    RG             = GL_RG,
    RGB            = GL_RGB,
    BGR            = GL_BGR,
    RGBA           = GL_RGBA,
    BGRA           = GL_BGRA,
    R_Integer      = GL_RED_INTEGER,
    RG_Integer     = GL_RG_INTEGER,
    RGB_Integer    = GL_RGB_INTEGER,
    BGR_Integer    = GL_BGR_INTEGER,
    RGBA_Integer   = GL_RGBA_INTEGER,
    BGRA_Integer   = GL_BGRA_INTEGER,
    StencilIndex   = GL_STENCIL_INDEX,
    DepthComponent = GL_DEPTH_COMPONENT,
    DepthStencil   = GL_DEPTH_STENCIL,
};

/// Type of the pixels used to create the texture
enum class Type : GLenum {
    UnsignedByte               = GL_UNSIGNED_BYTE,
    Byte                       = GL_BYTE,
    UnsignedShort              = GL_UNSIGNED_SHORT,
    Short                      = GL_SHORT,
    UnsignedInt                = GL_UNSIGNED_INT,
    Int                        = GL_INT,
    Float                      = GL_FLOAT,
    UnsignedByte_3_3_2         = GL_UNSIGNED_BYTE_3_3_2,
    UnsignedByte_2_3_3_Rev     = GL_UNSIGNED_BYTE_2_3_3_REV,
    UnsignedShort_5_6_5        = GL_UNSIGNED_SHORT_5_6_5,
    UnsignedShort_5_6_5_Rev    = GL_UNSIGNED_SHORT_5_6_5_REV,
    UnsignedShort_4_4_4_4      = GL_UNSIGNED_SHORT_4_4_4_4,
    UnsignedShort_4_4_4_4_Rev  = GL_UNSIGNED_SHORT_4_4_4_4_REV,
    UnsignedShort_5_5_5_1      = GL_UNSIGNED_SHORT_5_5_5_1,
    UnsignedShort_1_5_5_5_Rev  = GL_UNSIGNED_SHORT_1_5_5_5_REV,
    UnsignedInt_8_8_8_8        = GL_UNSIGNED_INT_8_8_8_8,
    UnsignedInt_8_8_8_8_Rev    = GL_UNSIGNED_INT_8_8_8_8_REV,
    UnsignedInt_10_10_10_2     = GL_UNSIGNED_INT_10_10_10_2,
    UnsignedInt_2_10_10_10_Rev = GL_UNSIGNED_INT_2_10_10_10_REV,
};

enum class Filter : GLint {
    NearestNeighbour   = GL_NEAREST,
    Linear             = GL_LINEAR,
    LinearMipmapLinear = GL_LINEAR_MIPMAP_LINEAR,
};

enum class Wrap : GLint {
    Repeat         = GL_REPEAT,
    MirroredRepeat = GL_MIRRORED_REPEAT,
    ClampToEdge    = GL_CLAMP_TO_EDGE,
    ClampToBorder  = GL_CLAMP_TO_BORDER,
};

namespace internal {
class UniqueTexture {
public:
    UniqueTexture() // NOLINT(*-member-init)
    {
        glGenTextures(1, &_id);
    }
    ~UniqueTexture()
    {
        glDeleteTextures(1, &_id);
    }
    UniqueTexture(UniqueTexture const&)                    = delete; // You cannot copy
    auto operator=(UniqueTexture const&) -> UniqueTexture& = delete; // a Texture. But you can move it, using std::move(my_texture)
    UniqueTexture(UniqueTexture&& o) noexcept
        : _id{o._id}
    {
        o._id = 0;
    }
    auto operator=(UniqueTexture&& o) noexcept -> UniqueTexture&
    {
        if (&o != this)
        {
            glDeleteTextures(1, &_id);
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

namespace TextureSource {
struct File {
    std::filesystem::path path{};
    bool                  flip_y{true}; /// There is often conflicting conventions between image files and OpenGL, they don't put the Y axis in the same direction. You can use this boolean to flip your image in the right direction.
    InternalFormat        texture_format{InternalFormat::RGBA};
};
struct Pixels {
    std::span<uint8_t const> pixels{};
    GLsizei                  width{};
    GLsizei                  height{};
    Type                     source_pixels_type{Type::UnsignedByte};
    Format                   source_pixels_format{Format::RGBA};
    InternalFormat           texture_format{InternalFormat::RGBA};
};
struct EmptyImage {
    GLsizei             width{};
    GLsizei             height{};
    InternalFormatSized texture_format{InternalFormatSized::RGBA8};
};
} // namespace TextureSource

using AnyTextureSource = std::variant<
    TextureSource::File,
    TextureSource::Pixels,
    TextureSource::EmptyImage>;

struct TextureOptions {
    Filter    minification_filter{Filter::Linear};
    Filter    magnification_filter{Filter::Linear};
    Wrap      wrap_x{Wrap::ClampToEdge};
    Wrap      wrap_y{Wrap::ClampToEdge};
    glm::vec4 border_color{0.f}; // Only used when at least one of the Wrap is set to ClampToBorder
};

class Texture {
public:
    explicit Texture(AnyTextureSource const&, TextureOptions const& = {});

    auto id() const -> GLuint { return _id.id(); }

private:
    internal::UniqueTexture _id{};
};

} // namespace gl