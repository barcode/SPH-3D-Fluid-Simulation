find_package(GLUT REQUIRED)
message(STATUS "GLUT_LIBRARIES ${GLUT_LIBRARIES}")
message(STATUS "GLUT_INCLUDE_DIR ${GLUT_INCLUDE_DIR}")

add_library(glvu STATIC
    glvu/camera.h
    glvu/glvu.cpp
    glvu/glvu.h
    glvu/glvuMat16fv.h
    glvu/glvuVec3f.h
    glvu/glvuVec3fv.h
    glvu/snapshot.h
    glvu/text.h
    glvu/trackball.h
)

target_include_directories(glvu PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${INSTALL_DIRECTORY_HEADERS}>
)
target_link_libraries(glvu GLUT::GLUT OpenGL GLU)
target_include_directories(glvu PUBLIC ${GLUT_INCLUDE_DIR})
