#pragma once
#include <filesystem>
#include <string>
#include <string_view>
#include <unordered_map>
#include <variant>
#include "Texture.hpp"
#include "glad/gl.h"
#include "glm/glm.hpp"

namespace gl {

namespace internal {
class UniqueShader {
public:
    UniqueShader()
        : _id{glCreateProgram()}
    {}
    ~UniqueShader()
    {
        glDeleteProgram(_id);
    }
    UniqueShader(UniqueShader const&)                    = delete; // You cannot copy
    auto operator=(UniqueShader const&) -> UniqueShader& = delete; // a Shader. But you can move it, using std::move(my_shader)
    UniqueShader(UniqueShader&& o) noexcept
        : _id{o._id}
    {
        o._id = 0;
    }
    auto operator=(UniqueShader&& o) noexcept -> UniqueShader&
    {
        if (&o != this)
        {
            glDeleteProgram(_id);
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

namespace ShaderSource {
struct File {
    std::filesystem::path path;
};
struct Code {
    std::string code;
};
} // namespace ShaderSource

using AnyShaderSource = std::variant<
    ShaderSource::File,
    ShaderSource::Code>;

struct Shader_Descriptor {
    AnyShaderSource vertex{};
    AnyShaderSource fragment{};
};

class Shader {
public:
    explicit Shader(Shader_Descriptor const&);

    auto id() const -> GLuint { return _id.id(); }

    void bind() const;
    void set_uniform(std::string_view uniform_name, int) const;
    void set_uniform(std::string_view uniform_name, unsigned int) const;
    void set_uniform(std::string_view uniform_name, bool) const;
    void set_uniform(std::string_view uniform_name, float) const;
    void set_uniform(std::string_view uniform_name, glm::vec2 const&) const;
    void set_uniform(std::string_view uniform_name, glm::vec3 const&) const;
    void set_uniform(std::string_view uniform_name, glm::vec4 const&) const;
    void set_uniform(std::string_view uniform_name, glm::uvec2 const&) const;
    void set_uniform(std::string_view uniform_name, glm::uvec3 const&) const;
    void set_uniform(std::string_view uniform_name, glm::uvec4 const&) const;
    void set_uniform(std::string_view uniform_name, glm::mat2 const&) const;
    void set_uniform(std::string_view uniform_name, glm::mat3 const&) const;
    void set_uniform(std::string_view uniform_name, glm::mat4 const&) const;
    void set_uniform(std::string_view uniform_name, Texture const&) const;

private:
    auto uniform_location(std::string_view uniform_name) const -> GLint;

private:
    internal::UniqueShader                         _id{};
    mutable std::unordered_map<std::string, GLint> _uniform_locations{};
};

} // namespace gl