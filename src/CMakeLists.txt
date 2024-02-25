# This is RBA
# 2012 -- 2014, Jan Heller, hellej1@cmp.felk.cvut.cz

ADD_EXECUTABLE(${PROJECT_NAME} 
camera.cpp
main.cpp
rba_calib.cpp
rba_data.cpp
rba_report.cpp
rdesc_mdh.cpp
robot.cpp
scene_calib.cpp
scene_load.cpp
scene_save.cpp
tdetector_opencv.cpp
# tdetector_elmark.cpp
# tdetector_lmcircles.cpp
# tdetector_lmchboard.cpp
# tdetector_lmcorners.cpp
# tdot.cpp
# mser.cpp
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OpenCV_LIBS})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${Boost_LIBRARIES})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${CERES_LIBRARIES})

IF (VLFEAT)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${VLFEAT_LIBRARY})
ENDIF ()

IF (CMAKE_THREAD_LIBS_INIT)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
ENDIF ()


# CImg 
INCLUDE_DIRECTORIES("../lib/cimg/")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Dcimg_display=0")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Dcimg_verbosity=0")

IF (OPENMP_FOUND)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Dcimg_use_openmp")
ENDIF ()

IF (PNG_FOUND)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${PNG_LIBRARIES})
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Dcimg_use_png")
ENDIF ()

IF (JPEG_FOUND)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${JPEG_LIBRARIES})
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Dcimg_use_jpeg")
ENDIF ()

IF (TIFF_FOUND)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${TIFF_LIBRARIES})
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Dcimg_use_tiff")
ENDIF ()

