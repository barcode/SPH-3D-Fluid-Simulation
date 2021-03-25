cmake_minimum_required(VERSION 3.10)

project(SPH-3D-Fluid-Simulation)

find_package(GLUT REQUIRED)
message(STATUS "GLUT_LIBRARIES ${GLUT_LIBRARIES}")
message(STATUS "GLUT_INCLUDE_DIR ${GLUT_INCLUDE_DIR}")

add_executable(simulation
    "594G Final Project/FIELD_2D.cpp"
    "594G Final Project/FIELD_2D.h"
    "594G Final Project/FIELD_3D.cpp"
    "594G Final Project/FIELD_3D.h"
    "594G Final Project/HW1.cpp"
    "594G Final Project/HW1.h"
    "594G Final Project/PARTICLE.cpp"
    "594G Final Project/PARTICLE.h"
    "594G Final Project/PARTICLE_SYSTEM.cpp"
    "594G Final Project/PARTICLE_SYSTEM.h"
    "594G Final Project/VEC3D.h"
    "594G Final Project/VEC3F.h"
    "594G Final Project/WALL.cpp"
    "594G Final Project/WALL.h"
    "594G Final Project/glvu/camera.h"
    "594G Final Project/glvu/glvu.cpp"
    "594G Final Project/glvu/glvu.h"
    "594G Final Project/glvu/glvuMat16fv.h"
    "594G Final Project/glvu/glvuVec3f.h"
    "594G Final Project/glvu/glvuVec3fv.h"
    "594G Final Project/glvu/snapshot.h"
    "594G Final Project/glvu/text.h"
    "594G Final Project/glvu/trackball.h"
)

target_link_libraries(simulation GLUT::GLUT OpenGL GLU)
target_include_directories(simulation PRIVATE ${GLUT_INCLUDE_DIR})
