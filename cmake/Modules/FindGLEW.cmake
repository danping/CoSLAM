#
# Try to find GLEW library and include path.
# Once done this will define
#
# GLEW_FOUND
# GLEW_INCLUDE_PATH
# GLEW_LIBRARY
# 

IF (WIN32)
	FIND_PATH( GLEW_INCLUDE_PATH GL/glew.h
		$ENV{PROGRAMFILES}/GLEW/include
		${PROJECT_SOURCE_DIR}/src/nvgl/glew/include
		DOC "The directory where GL/glew.h resides")
	FIND_LIBRARY( GLEW_LIBRARY
		NAMES glew GLEW glew32 glew32s
		PATHS
		$ENV{PROGRAMFILES}/GLEW/lib
		${PROJECT_SOURCE_DIR}/src/nvgl/glew/bin
		${PROJECT_SOURCE_DIR}/src/nvgl/glew/lib
		DOC "The GLEW library")
ELSE (WIN32)
	FIND_PATH( GLEW_INCLUDE_PATH GL/glew.h
		/usr/include
		/usr/local/include
		/sw/include
		/opt/local/include
		DOC "GL/glew.h detected!")
	FIND_LIBRARY( GLEW_LIBRARY
		NAMES GLEW glew
		PATHS
		/usr/lib64
		/usr/lib
		/usr/local/lib64
		/usr/local/lib
		/sw/lib
		/opt/local/lib
		DOC "GLEW library detected!")
ENDIF (WIN32)

message(status, "libraries:" ${GLEW_LIBRARY})

IF (GLEW_INCLUDE_PATH AND GLEW_LIBRARY)
    SET( GLEW_FOUND TRUE CACHE STRING "GLEW library has been detected!")
ELSE (GLEW_INCLUDE_PATH AND GLEW_LIBRARY)
    SET( GLEW_FOUND FALSE CACHE STRING "fail to detect GLEW library!")
ENDIF (GLEW_INCLUDE_PATH AND GLEW_LIBRARY)

MARK_AS_ADVANCED( GLEW_FOUND GLEW_INCLUDE_PATH GLEW_LIBRARY)

