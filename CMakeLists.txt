cmake_minimum_required(VERSION 3.15)
project(FactoryFlow3D)

set(CMAKE_CXX_STANDARD 17)

include(FetchContent)

# GLFW
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.3.8
)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glfw)

# GLM
FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG 0.9.9.8
)
FetchContent_MakeAvailable(glm)

include_directories(include)

add_executable(FactoryFlow3D
    src/main.cpp
    src/glad.c
)

target_link_libraries(FactoryFlow3D glfw glm)
# Link OpenGL on Windows
if(WIN32)
    target_link_libraries(FactoryFlow3D opengl32)
endif()
