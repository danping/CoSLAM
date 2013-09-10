FIND_PATH( VisualSLAM_INCLUDE_PATH VisualSLAM/SL_error.h
            /usr/include
            /usr/local/include
            /usr/share/local/include
            /sw/include
            /opt/local/include) 


FIND_LIBRARY(VisualSLAM_LIBRARY
            NAMES VisualSLAM 
            PATHS
            /usr/lib64
            /usr/lib
            /usr/local/lib64
            /usr/local/lib
            /sw/lib
            /opt/local/lib
) 

message(STATUS "libVisuaSLAM header files:" ${VisualSLAM_INCLUDE_PATH})
message(STATUS "libVisuaSLAM lib files:" ${VisualSLAM_LIBRARY})

IF( VisualSLAM_INCLUDE_PATH AND VisualSLAM_LIBRARY)
    SET( VisualSLAM_FOUND "Yes")
ELSE(VisualSLAM_INCLUDE_PATH AND VisualSLAM_LIBRARY)
    SET( VisualSLAM_FOUND "No")
    message(STATUS "libVisualSLAM - not found!")
ENDIF( VisualSLAM_INCLUDE_PATH AND VisualSLAM_LIBRARY)
    

MARK_AS_ADVANCED( 
    VisualSLAM_FOUND 
    VisualSLAM_INCLUDE_PATH
    VisualSLAM_LIBRARY
)

