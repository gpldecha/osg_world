cmake_minimum_required(VERSION 2.8.3)
project(test_open_sg)


set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -isystem -mtune=native -fPIC -Wall -Wno-deprecated-declarations -Wno-reorder")
set(CMAKE_BUILD_TYPE Release                                                                    )
set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG}"                                          )
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}"                                        )

# https://vicrucann.github.io/tutorials/osg-linux-quick-install/

find_package( OpenCV REQUIRED )
#find_package(Boost  1.57.0 REQUIRED COMPONENTS system thread)


set(OSG_LIGB
    osg
    osgAnimation
    osgDB
    osgFX
    osgGA
    osgManipulator
    osgParticle
    osgPresentation
    osgShadow
    osgSim
    osgTerrain
    osgText
    osgUtil
    osgViewer
    osgVolume
    OpenThreads)


###########
## Build ##
###########

set(MJPEG_SERVER_LIB /usr/local/lib/libmjpeg_server.so)
set(MJPEG_SERVER_INC /usr/local/include/mjpeg_server)


include_directories(include ${catkin_INCLUDE_DIRS} ${OPENSCENEGRAPH_INCLUDE_DIR} ${MJPEG_SERVER_INC})


set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)


set(SRC_FILES main.cpp)


add_executable(world_sim ${SRC_FILES})
target_link_libraries(world_sim ${catkin_LIBRARIES} GL GLU glut ${OSG_LIGB})


add_executable(test_home Testhome.cpp KeyboardHandler.cpp TransformAccumulator.cpp )
target_link_libraries(test_home boost_thread boost_system boost_regex ${catkin_LIBRARIES}  ${Boost_LIBRARIES} GL GLU glut ${OSG_LIGB} ${OpenCV_LIBS} ${MJPEG_SERVER_LIB})


add_executable(test_reference plot_reference_frame.cpp)
target_link_libraries(test_reference boost_thread boost_system boost_regex ${catkin_LIBRARIES}  ${Boost_LIBRARIES} GL GLU glut ${OSG_LIGB} ${OpenCV_LIBS} ${MJPEG_SERVER_LIB})


#add_executable(test_matrix TestMatrix.cpp KeyboardHandler.cpp)
#target_link_libraries(test_matrix ${catkin_LIBRARIES} GL GLU glut ${OSG_LIGB} ${OpenCV_LIBS})

#add_executable(take_pictures take_picture.cpp)
#target_link_libraries(take_pictures ${catkin_LIBRARIES} GL GLU glut ${OSG_LIGB} ${OpenCV_LIBS})


#add_executable(osgprerender osgprerender.cpp)
#target_link_libraries(osgprerender ${catkin_LIBRARIES} GL GLU glut ${OSG_LIGB} ${OpenCV_LIBS})



