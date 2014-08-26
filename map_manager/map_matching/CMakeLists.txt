#ADD_LIBRARY(ransac_matching SHARED
#ransac.cpp ransac.h
#)
#SET_TARGET_PROPERTIES(ransac_matching PROPERTIES OUTPUT_NAME ${LIB_PREFIX})
#TARGET_LINK_LIBRARIES(ransac_matching 
#  ${OpenCV_LIBS}2
#)

ADD_EXECUTABLE(test_homographyRansac test_homographyRansac.cpp)
TARGET_LINK_LIBRARIES(test_homographyRansac
  #ransac_matching
  ${OpenCV_LIBS}
)
