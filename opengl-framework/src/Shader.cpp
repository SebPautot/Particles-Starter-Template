#include "Shader.hpp"
#include <cassert>
#include <fstream>
#include "Texture.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "handle_error.hpp"
#include "make_absolute_path.hpp"

namespace {

void compile_shader_module(GLuint id, std::string const& source_code)
{
    char const* src = source_code.c_str();
    glShaderSource(id, 1, &src, nullptr);
    glCompileShader(id);

    { // Check for errors
        int result;
        glGetShaderiv(id, GL_COMPILE_STATUS, &result);
        if (result)
            return; // Compilation successful

        GLsizei length;
        glGetShaderiv(id, GL_INFO_LOG_LENGTH, &length);
        std::vector<GLchar> error_message;
        error_message.resize(static_cast<size_t>(length));
        glGetShaderInfoLog(id, length, nullptr, error_message.data());
        gl::handle_error(std::format("Shader Compilation failed:\n{}\n\nThe code we tried to compile was:\n{}", error_message.data(), source_code));
    }
}

auto get_source_code(gl::ShaderSource::Code const& source) -> std::string
{
    return source.code;
}
auto get_source_code(gl::ShaderSource::File const& source) -> std::string
{
    auto ifs = std::ifstream{gl::make_absolute_path(source.path)};
    return std::string{std::istreambuf_iterator<char>{ifs}, {}};
}

class UniqueShaderModule {
public:
    explicit UniqueShaderModule(GLenum shader_kind, gl::AnyShaderSource const& source)
        : _id{glCreateShader(shader_kind)}
    {
        compile_shader_module(_id, std::visit([](auto&& source) { return get_source_code(source); }, source));
    }
    ~UniqueShaderModule()
    {
        glDeleteShader(_id);
    }
    UniqueShaderModule(UniqueShaderModule const&)                    = delete;
    auto operator=(UniqueShaderModule const&) -> UniqueShaderModule& = delete;
    UniqueShaderModule(UniqueShaderModule&& o) noexcept
        : _id{o._id}
    {
        o._id = 0;
    }
    auto operator=(UniqueShaderModule&& o) noexcept -> UniqueShaderModule&
    {
        if (&o != this)
        {
            glDeleteShader(_id);
            _id   = o._id;
            o._id = 0;
        }
        return *this;
    }

    auto id() const { return _id; }

private:
    GLuint _id;
};

void check_for_linking_errors(GLuint shader_id)
{
    int result;
    glGetProgramiv(shader_id, GL_LINK_STATUS, &result);
    if (result == GL_FALSE)
    {
        GLsizei length;
        glGetProgramiv(shader_id, GL_INFO_LOG_LENGTH, &length);
        std::vector<GLchar> error_message;
        error_message.resize(static_cast<size_t>(length));
        glGetProgramInfoLog(shader_id, length, nullptr, error_message.data());
        gl::handle_error(std::format("Shader Linking failed:\n{}", error_message.data()));
    }
}

} // namespace

namespace gl {

Shader::Shader(Shader_Descriptor const& desc)
{
    auto vertex_shader   = UniqueShaderModule{GL_VERTEX_SHADER, desc.vertex};
    auto fragment_shader = UniqueShaderModule{GL_FRAGMENT_SHADER, desc.fragment};
    glAttachShader(id(), vertex_shader.id());
    glAttachShader(id(), fragment_shader.id());
    glLinkProgram(id());
    glDetachShader(id(), fragment_shader.id());
    glDetachShader(id(), vertex_shader.id());
    check_for_linking_errors(id());
}

static void assert_shader_is_bound(GLuint id)
{
#ifndef NDEBUG
    GLint current_id;
    glGetIntegerv(GL_CURRENT_PROGRAM, &current_id);
    assert(static_cast<GLuint>(current_id) == id && "You must call shader.bind() before setting any uniform.");
#else
    std::ignore = id;
#endif
}

void Shader::bind() const
{
    glUseProgram(id());
}

auto Shader::uniform_location(std::string_view uniform_name) const -> GLint
{
    auto const name = std::string{uniform_name};
    auto const it   = _uniform_locations.find(name);
    if (it != _uniform_locations.end())
    {
        return it->second;
    }
    else
    {
        GLint const location     = glGetUniformLocation(id(), name.c_str());
        _uniform_locations[name] = location;
        return location;
    }
}

void Shader::set_uniform(std::string_view uniform_name, int v) const
{
    assert_shader_is_bound(id());
    glUniform1i(uniform_location(uniform_name), v);
}
void Shader::set_uniform(std::string_view uniform_name, unsigned int v) const
{
    set_uniform(uniform_name, static_cast<int>(v));
}
void Shader::set_uniform(std::string_view uniform_name, bool v) const
{
    set_uniform(uniform_name, v ? 1 : 0);
}
void Shader::set_uniform(std::string_view uniform_name, float v) const
{
    assert_shader_is_bound(id());
    glUniform1f(uniform_location(uniform_name), v);
}
void Shader::set_uniform(std::string_view uniform_name, const glm::vec2& v) const
{
    assert_shader_is_bound(id());
    glUniform2f(uniform_location(uniform_name), v.x, v.y);
}
void Shader::set_uniform(std::string_view uniform_name, const glm::vec3& v) const
{
    assert_shader_is_bound(id());
    glUniform3f(uniform_location(uniform_name), v.x, v.y, v.z);
}
void Shader::set_uniform(std::string_view uniform_name, const glm::vec4& v) const
{
    assert_shader_is_bound(id());
    glUniform4f(uniform_location(uniform_name), v.x, v.y, v.z, v.w);
}
void Shader::set_uniform(std::string_view uniform_name, const glm::uvec2& v) const
{
    assert_shader_is_bound(id());
    glUniform2ui(uniform_location(uniform_name), v.x, v.y);
}
void Shader::set_uniform(std::string_view uniform_name, const glm::uvec3& v) const
{
    assert_shader_is_bound(id());
    glUniform3ui(uniform_location(uniform_name), v.x, v.y, v.z);
}
void Shader::set_uniform(std::string_view uniform_name, const glm::uvec4& v) const
{
    assert_shader_is_bound(id());
    glUniform4ui(uniform_location(uniform_name), v.x, v.y, v.z, v.w);
}
void Shader::set_uniform(std::string_view uniform_name, const glm::mat2& mat) const
{
    assert_shader_is_bound(id());
    glUniformMatrix2fv(uniform_location(uniform_name), 1, GL_FALSE, glm::value_ptr(mat));
}
void Shader::set_uniform(std::string_view uniform_name, const glm::mat3& mat) const
{
    assert_shader_is_bound(id());
    glUniformMatrix3fv(uniform_location(uniform_name), 1, GL_FALSE, glm::value_ptr(mat));
}
void Shader::set_uniform(std::string_view uniform_name, const glm::mat4& mat) const
{
    assert_shader_is_bound(id());
    glUniformMatrix4fv(uniform_location(uniform_name), 1, GL_FALSE, glm::value_ptr(mat));
}

static auto max_number_of_texture_slots() -> GLuint
{
    GLint res{};
    glGetIntegerv(GL_MAX_TEXTURE_IMAGE_UNITS, &res);
    return static_cast<GLuint>(res);
}

static auto get_next_texture_slot() -> GLuint
{
    static GLuint       current_slot = 0;
    static GLuint const max_slots    = max_number_of_texture_slots();

    current_slot = (current_slot + 1) % max_slots;
    if (current_slot == 0) // HACK Slot 0 is used for texture operations like resizing and setting the image, anyone might override the texture set here at any time. So we use all slots but the 0th one for rendering.
        current_slot = 1;
    return current_slot;
}

void Shader::set_uniform(std::string_view uniform_name, Texture const& texture) const
{
    auto const slot = get_next_texture_slot();
    glActiveTexture(GL_TEXTURE0 + slot);
    glBindTexture(GL_TEXTURE_2D, texture.id());
    set_uniform(uniform_name, slot);
    glActiveTexture(GL_TEXTURE0); // HACK Slot 0 is used for texture operations like resizing and setting the image, anyone might override the texture set here at any time. So we use all slots but the 0th one for rendering.
}

// void Shader::set_uniform_texture(std::string_view uniform_name, GLuint texture_id, TextureSamplerDescriptor const& sampler) const
// {
//     auto const slot = get_next_texture_slot();
//     glActiveTexture(GL_TEXTURE0 + slot);
//     glBindTexture(GL_TEXTURE_2D, texture_id);
//     glBindSampler(slot, TextureSamplerLibrary::instance().get(sampler).id()));
//     set_uniform(uniform_name, static_cast<int>(slot));
//     glActiveTexture(GL_TEXTURE0); // HACK Slot 0 is used for texture operations like resizing and setting the image, anyone might override the texture set here at any time. So we use all slots but the 0th one for rendering.
// }

} // namespace gl
