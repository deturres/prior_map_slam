#ADD_LIBRARY(boss_map_reading SHARED
#)
#SET_TARGET_PROPERTIES(boss_map_reading PROPERTIES OUTPUT_NAME ${LIB_PREFIX})
#TARGET_LINK_LIBRARIES(boss_map_reading 
#  ${OpenCV_LIBS} 
#  ${BOSS_LIBRARY}
#)

ADD_EXECUTABLE(map_processing map_processing.cpp)
TARGET_LINK_LIBRARIES(map_processing
  ${OpenCV_LIBS}
)
