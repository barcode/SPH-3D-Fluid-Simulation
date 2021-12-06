find_package(
    Boost
    COMPONENTS
        system
        filesystem
    REQUIRED)
find_package (Threads)







add_library(cylindrical_wall_simulation_webserver STATIC
    cylindrical_wall_simulation_webserver.cpp
    cylindrical_wall_simulation_webserver.h
)

target_include_directories(cylindrical_wall_simulation_webserver PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${INSTALL_DIRECTORY_HEADERS}>
)

target_link_libraries(cylindrical_wall_simulation_webserver
    PUBLIC

    Boost::headers
    Boost::system
    Boost::filesystem
    jpeg
    dl
    ${CMAKE_THREAD_LIBS_INIT}
    cylindrical_wall_simulation)

target_compile_options(cylindrical_wall_simulation_webserver PUBLIC -pthread)

configure_file(configure.html "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/configure.html" COPYONLY)



