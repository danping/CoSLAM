project(CoSLAM)
cmake_minimum_required(VERSION 2.8)

#search extern libraries
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

find_package(VisualSLAM REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)
find_package(CG REQUIRED)
find_package(BLAS REQUIRED)

find_package(LAPACK REQUIRED)
find_package(OpenCV REQUIRED)
find_package(wxWidgets COMPONENTS core base gl REQUIRED)

aux_source_directory(src/app SRC_APP)
aux_source_directory(src/gui SRC_GUI)
aux_source_directory(src/slam SRC_SLAM)
aux_source_directory(src/tracking SRC_TRACKING)
aux_source_directory(src/tracking/CGKLT SRC_CGKLT)

include_directories(src)
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} ${GLEW_INCLUDE_PATH} ${CG_INCLUDE_PATH} ${OpenCV_INCLUDE_DIRS})
include_directories("${VisualSLAM_INCLUDE_PATH}/VisualSLAM/")
include( "${wxWidgets_USE_FILE}" )

add_definitions(-DV3DLIB_ENABLE_GPGPU -DV3DLIB_GPGPU_ENABLE_CG)
add_executable(${PROJECT_NAME} ${SRC_APP} ${SRC_GUI} ${SRC_SLAM} ${SRC_TRACKING} ${SRC_CGKLT})

target_link_libraries(${PROJECT_NAME}
 ${wxWidgets_LIBRARIES}
 ${GLEW_LIBRARY}
 ${OpenCV_LIBS}
 ${GLUT_LIBRARY}
 ${VisualSLAM_LIBRARY}
 ${BLAS_LIBRARIES}
 ${LAPACK_LIBRARIES}
 ${OPENGL_glu_LIBRARY}
 ${OPENGL_gl_LIBRARY}
 ${CG_LIBRARY}
 ${CG_GL_LIBRARY}
)

install (TARGETS ${PROJECT_NAME} DESTINATION bin)

FILE(GLOB shaders "src/tracking/CGKLT/Shaders/*.cg")

install(FILES
    ${shaders}
    DESTINATION
    /usr/share/CoSLAM
)

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
