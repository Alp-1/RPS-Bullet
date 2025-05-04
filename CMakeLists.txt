cmake_minimum_required(VERSION 3.16)
project(RPS LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Bullet MODULE REQUIRED)
find_package(OpenGL REQUIRED)           # defines OPENGL_LIBRARIES & OPENGL_glu_LIBRARY
find_package(GLEW REQUIRED)
find_package(glfw3 3.3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(HDF5 REQUIRED COMPONENTS C HL)
find_package(Protobuf REQUIRED)

add_executable(RPS src/main.cpp)

target_include_directories(RPS PRIVATE
        ${BULLET_INCLUDE_DIRS}
        ${OPENGL_INCLUDE_DIRS}
        ${GLEW_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${HDF5_INCLUDE_DIRS}
)

target_link_libraries(RPS PRIVATE
        ${BULLET_LIBRARIES}
        OpenGL::GL
        ${OPENGL_glu_LIBRARY}      # <— add this
        GLEW::GLEW
        glfw
        ${OpenCV_LIBS}
        ${HDF5_LIBRARIES}
        ${HDF5_HL_LIBRARIES}
        protobuf::libprotobuf
)
