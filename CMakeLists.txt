cmake_minimum_required(VERSION 3.20)
project(Particles)

file(GLOB_RECURSE SOURCE_FILES CONFIGURE_DEPENDS src/*)
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_include_directories(${PROJECT_NAME} PRIVATE src)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)

# Include lib
add_subdirectory(opengl-framework)
target_link_libraries(${PROJECT_NAME} PRIVATE opengl_framework::opengl_framework)
gl_target_copy_folder(${PROJECT_NAME} res)