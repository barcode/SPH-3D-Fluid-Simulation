cmake_minimum_required(VERSION 3.7)

project(SPH-3D-Fluid-Simulation)

################################################################################
#setup cmake variables
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
################################################################################
#add subdirs
add_subdirectory(src/src_glvu)
add_subdirectory(src/src_particle_simulation)
add_subdirectory(src/src_cylindrical_wall_simulation)
add_subdirectory(src/src_cylindrical_wall_simulation_webserver)

add_subdirectory(src/src_simulator_qt_widget)
add_subdirectory(src/src_simulator_gl)
add_subdirectory(src/src_simulator_gl_using_cylindrical_wall_simulator)
add_subdirectory(src/src_simulator_qt)
add_subdirectory(src/src_simulator_both)
add_subdirectory(src/src_simulator_webserver)

