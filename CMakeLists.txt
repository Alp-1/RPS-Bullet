cmake_minimum_required(VERSION 3.28)
project(SFML_Bullet_ImGui LANGUAGES CXX)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# SFML
FetchContent_Declare(SFML
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG 3.0.1
    GIT_SHALLOW ON
    EXCLUDE_FROM_ALL
    SYSTEM)
FetchContent_MakeAvailable(SFML)

# Bullet Physics
FetchContent_Declare(bullet
    GIT_REPOSITORY https://github.com/bulletphysics/bullet3.git
    GIT_TAG master
    GIT_SHALLOW ON
    EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(bullet)


# Executable
add_executable(main src/main.cpp)
target_compile_features(main PRIVATE cxx_std_17)

target_include_directories(main PRIVATE
    ${bullet_SOURCE_DIR}/src
)

target_link_libraries(main
    PRIVATE
        SFML::Graphics
        BulletDynamics
        BulletCollision
        LinearMath
)
