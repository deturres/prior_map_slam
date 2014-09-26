# Find the header files

FIND_PATH(G2O_FRONTEND_INCLUDE_DIR
  ${PROJECT_SOURCE_DIR}/../g2o_frontend
  ${PROJECT_SOURCE_DIR}/../../g2o_frontend
  ${G2O_FRONTEND_ROOT}/include
  $ENV{G2O_FRONTEND_ROOT}/include
  $ENV{G2O_FRONTEND_ROOT}
  /usr/local/include
  /usr/include
  /opt/local/include
  /sw/local/include
  /sw/include
  NO_DEFAULT_PATH
  )
# Ugly but FIND_PATH is not working..???
SET(G2O_FRONTEND_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/../../g2o_frontend)


# Macro to unify finding both the debug and release versions of the
# libraries; this is adapted from the OpenSceneGraph FIND_LIBRARY
# macro.

MACRO(FIND_G2O_FRONTEND_LIBRARY MYLIBRARY MYLIBRARYNAME)

  FIND_LIBRARY("${MYLIBRARY}_DEBUG"
    NAMES "g2o_frontend_${MYLIBRARYNAME}_d"
    PATHS
    ${G2O_FRONTENED_ROOT}/lib/Debug
    ${G2O_FRONTENED_ROOT}/lib
    $ENV{G2O_FRONTENED_ROOT}/lib/Debug
    $ENV{G2O_FRONTENED_ROOT}/lib
    NO_DEFAULT_PATH
    )

  FIND_LIBRARY("${MYLIBRARY}_DEBUG"
    NAMES "g2o_frontend_${MYLIBRARYNAME}_d"
    PATHS
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64
    /opt/local/lib
    /sw/local/lib
    /sw/lib
    )
  
  FIND_LIBRARY(${MYLIBRARY}
    NAMES "g2o_frontend_${MYLIBRARYNAME}"
    PATHS
    ${G2O_FRONTEND_ROOT}/lib/Release
    ${G2O_FRONTEND_ROOT}/lib
    $ENV{G2O_FRONTENED_ROOT}/lib/Release
    $ENV{G2O_FRONTENED_ROOT}/lib
    NO_DEFAULT_PATH
    )

  FIND_LIBRARY(${MYLIBRARY}
    NAMES "g2o_frontend_${MYLIBRARYNAME}"
    PATHS
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64
    /opt/local/lib
    /sw/local/lib
    /sw/lib
    )
  
  IF(NOT ${MYLIBRARY}_DEBUG)
    IF(MYLIBRARY)
      SET(${MYLIBRARY}_DEBUG ${MYLIBRARY})
    ENDIF(MYLIBRARY)
  ENDIF( NOT ${MYLIBRARY}_DEBUG)
  
ENDMACRO(FIND_G2O_FRONTEND_LIBRARY LIBRARY LIBRARYNAME)

# Find the core elements (when needed)
FIND_G2O_FRONTEND_LIBRARY(G2O_FRONTEND_BASEMATH basemath)
FIND_G2O_FRONTEND_LIBRARY(G2O_FRONTEND_SENSOR_DATA sensor_data)

# G2O itself declared found if we found the core libraries and at least one solver
SET(G2O_FRONTEND_FOUND "NO")
IF(G2O_FRONTEND_INCLUDE_DIR AND G2O_FRONTEND_BASEMATH AND G2O_FRONTEND_SENSOR_DATA)
  SET(G2O_FRONTEND_FOUND "YES")
ENDIF(G2O_FRONTEND_INCLUDE_DIR AND G2O_FRONTEND_BASEMATH AND G2O_FRONTEND_SENSOR_DATA)
