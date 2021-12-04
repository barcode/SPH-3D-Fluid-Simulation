add_library(cylindrical_wall_simulation STATIC
    cylindrical_wall_simulation.cpp
    cylindrical_wall_simulation.h
)

target_include_directories(cylindrical_wall_simulation PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${INSTALL_DIRECTORY_HEADERS}>
)

target_link_libraries(cylindrical_wall_simulation particle_simulation)

