cmake_minimum_required(VERSION 2.8.3)
project(test_multiplecameras)


set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -isystem -mtune=native -fPIC -Wall -Wno-deprecated-declarations -Wno-reorder")
set(CMAKE_BUILD_TYPE Release                                                                    )
set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG}"                                          )
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}"                                        )

# https://vicrucann.github.io/tutorials/osg-linux-quick-install/

set(OSG_LIGB
    osg
    osgDB
    osgUtil
    osgGA
    osgSim
    osgViewer
    osgText
    osgAnimation
    osgManipulator
    osgTerrain
    osgFX
    osgShadow
    OpenThreads)


###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${OPENSCENEGRAPH_INCLUDE_DIR})


set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)


set(SRC_FILES findNodeVisitor.cpp FollowNodeManip.cpp TestMultiCamera.cpp TransformAccumulator.cpp createView.cpp)

add_executable(world_sim ${SRC_FILES})
target_link_libraries(world_sim ${catkin_LIBRARIES} GL GLU glut ${OSG_LIGB})


