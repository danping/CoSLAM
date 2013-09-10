
 
FIND_PATH( CG_INCLUDE_PATH
            Cg/cg.h
            /usr/include
            /usr/local/include
            /usr/share/local/include
            /sw/include
            /opt/local/include
            DOC "Nvidia Cg toolkit - Header files are detected!"
) 

FIND_LIBRARY(CG_LIBRARY
            NAMES Cg CG
            PATHS
            /usr/lib64
            /usr/lib
            /usr/local/lib64
            /usr/local/lib
            /sw/lib
            /opt/local/lib
)   

FIND_LIBRARY(CG_GL_LIBRARY
            NAMES CgGL cggl CGGL
            PATHS
            /usr/lib64
            /usr/lib
            /usr/local/lib64
            /usr/local/lib
            /sw/lib
            /opt/local/lib
)   

message(STATUS "cg include path:" ${CG_INCLUDE_PATH})
message(STATUS "cg libs:" ${CG_LIBRARY})
message(STATUS "cggl libs:" ${CG_GL_LIBRARY})
message(STATUS "Visual slam:" ${VisualSLAM_LIBRARY})

IF( CG_INCLUDE_PATH AND CG_LIBRARY AND CG_GL_LIBRARY)
    SET( CG_FOUND TURE)
ELSE( CG_INCLUDE_PATH AND CG_LIBRARY AND CG_GL_LIBRARY)
    SET( CG_FOUND FALSE)
ENDIF( CG_INCLUDE_PATH AND CG_LIBRARY AND CG_GL_LIBRARY)
    

MARK_AS_ADVANCED( 
    CG_FOUND 
    CG_INCLUDE_PATH
    CG_LIBRARY
    CG_GL_LIBRARY
)