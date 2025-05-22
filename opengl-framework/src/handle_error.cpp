#include "handle_error.hpp"
#include <iostream>
#include <stdexcept>

namespace gl {

void handle_error(std::string const& error_message)
{
    std::cerr << error_message << '\n';
    throw std::runtime_error{error_message};
}

} // namespace gl