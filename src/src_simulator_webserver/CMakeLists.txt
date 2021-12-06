add_executable(simulator_webserver simulator_webserver.cpp)
target_link_libraries(
    simulator_webserver
    cylindrical_wall_simulation_webserver
)
