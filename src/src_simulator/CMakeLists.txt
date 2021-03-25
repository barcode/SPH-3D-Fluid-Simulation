add_executable(simulator
    FIELD_2D.cpp
    FIELD_2D.h
    FIELD_3D.cpp
    FIELD_3D.h
    HW1.cpp
    HW1.h
    PARTICLE.cpp
    PARTICLE.h
    PARTICLE_SYSTEM.cpp
    PARTICLE_SYSTEM.h
    VEC3D.h
    VEC3F.h
    WALL.cpp
    WALL.h
)

target_include_directories(simulator PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${INSTALL_DIRECTORY_HEADERS}>
)

target_link_libraries(simulator glvu)
