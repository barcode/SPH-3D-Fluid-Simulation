add_executable(simulator_gl_using_cylindrical_wall_simulator simulator_gl_using_cylindrical_wall_simulator.cpp)
target_link_libraries(simulator_gl_using_cylindrical_wall_simulator cylindrical_wall_simulation)
if(NOT TARGET glvu)
    set_target_properties(simulator_gl_using_cylindrical_wall_simulator PROPERTIES EXCLUDE_FROM_ALL ON)
endif()
