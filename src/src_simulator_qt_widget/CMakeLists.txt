find_package(
    Qt5
    COMPONENTS
        Core
        Gui
        Widgets
        3DCore
        3DExtras
        3DRender
        3DInput
    REQUIRED)

if(Qt5_FOUND)
    set(CMAKE_AUTOUIC ON)
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTORCC ON)
    set(CMAKE_INCLUDE_CURRENT_DIR ON)

    add_library(simulator_qt_widget
        STATIC
        qt_sim_window.h
        qt_sim_window.cpp
        qt_sim_window.ui
    )

    target_include_directories(simulator_qt_widget PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${INSTALL_DIRECTORY_HEADERS}>
    )
    target_link_libraries(
        simulator_qt_widget
        cylindrical_wall_simulation
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
        Qt5::3DCore
        Qt5::3DExtras
        Qt5::3DRender
        Qt5::3DInput
    )
endif()
