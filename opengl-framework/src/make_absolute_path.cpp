#include "make_absolute_path.hpp"
#include <filesystem>
#include "exe_path/exe_path.h"
#include "handle_error.hpp"

namespace gl {

auto make_absolute_path(std::filesystem::path const& path) -> std::filesystem::path
{
    auto absolute_path = path.is_relative()
                             ? exe_path::dir() / path
                             : path;

    if (!std::filesystem::exists(absolute_path))
        handle_error(std::format("Path \"{}\" does not exist. Make sure you didn't misspell it or made it relative to the wrong folder.", absolute_path.string()));

    return absolute_path;
}

} // namespace gl
