INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR} ${G2O_INCLUDE_DIR} ${G2O_FRONTEND_INCLUDE_DIR})
#ADD_LIBRARY(ransac_matching SHARED 
					#ransac.cpp ransac.h
#)
#SET_TARGET_PROPERTIES(ransac_matching PROPERTIES OUTPUT_NAME ${LIB_PREFIX})
#TARGET_LINK_LIBRARIES(ransac_matching 
#  ${OpenCV_LIBS}
#)

ADD_EXECUTABLE(test_homographyRansac 
		test_homographyRansac.cpp utility.cpp)
TARGET_LINK_LIBRARIES(test_homographyRansac
  #ransac_matching
  ${OpenCV_LIBS}
)


ADD_EXECUTABLE(graph_addconstraints 
		graph_add_constraints.cpp utility.cpp)
TARGET_LINK_LIBRARIES(graph_addconstraints
		${OpenCV_LIBS} ${G2O_TYPES_DATA} ${G2O_FRONTEND_SENSOR_DATA}
		${G2O_TYPES_SLAM2D} ${G2O_TYPES_SLAM2D_ADDONS} 
		${G2O_TYPES_SLAM3D} ${G2O_TYPES_SLAM3D_ADDONS} 
		${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CSPARSE_EXTENSION} 
		${G2O_CORE_LIBRARY} ${G2O_OPENGL_HELPER_LIBRARY} ${CSPARSE_LIBRARY}
)

