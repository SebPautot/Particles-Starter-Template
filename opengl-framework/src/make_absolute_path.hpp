#pragma once
#include <filesystem>

namespace gl {

auto make_absolute_path(std::filesystem::path const& path) -> std::filesystem::path;

} // namespace gl
