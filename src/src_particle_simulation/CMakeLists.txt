add_library(particle_simulation STATIC
    FIELD_3D.cpp
    FIELD_3D.h
    PARTICLE.cpp
    PARTICLE.h
    PARTICLE_SYSTEM.cpp
    PARTICLE_SYSTEM.h
    VEC3.h
    WALL.h
    CYLINDRICAL_WALL.h
)

target_include_directories(particle_simulation PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${INSTALL_DIRECTORY_HEADERS}>
)
if(TARGET glvu)
    target_link_libraries(particle_simulation glvu)
else()
    target_compile_options(particle_simulation PUBLIC -DNO_OPENGL)
endif()
