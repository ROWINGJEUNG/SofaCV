cmake_minimum_required(VERSION 3.12)
project(SofaCV VERSION 23.0814)

find_package(OpenCV REQUIRED)

find_package(Sofa.Core REQUIRED)
sofa_find_package(Sofa.Framework)
sofa_find_package(Sofa.Simulation REQUIRED)
sofa_find_package(Sofa.Component REQUIRED)
sofa_find_package(Sofa.GL REQUIRED)
sofa_find_package(Sofa.GUI REQUIRED)
sofa_find_package(CImgPlugin REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(RealSense_INSTALL_PATH "C:/Program Files (x86)/Intel RealSense SDK 2.0" )
set(RealSense_LIBS "${RealSense_INSTALL_PATH}/lib/x64/realsense2.lib" )
set(RealSense_INCLUDE_DIR "${RealSense_INSTALL_PATH}/include" )
include_directories(${RealSense_INCLUDE_DIR})

set(CONFIG_FILES
  src/SofaCV/SofaCVPlugin.h
  src/SofaCV/SofaCVPlugin.cpp
  )

set(HEADER_FILES
  src/SofaCV/SofaCV.h
  src/SofaCV/PCA_jdg.h
  src/SofaCV/ImplicitDataEngine.h
  src/SofaCV/ImageFilter.h
  src/SofaCV/FemurConstraintForce.h
  src/SofaCV/DGConstraint.h

  src/SofaCV/acquisition/BaseFrameGrabber.h
  src/SofaCV/acquisition/BaseGrabber.h
  src/SofaCV/acquisition/BaseScheduler.h
  src/SofaCV/acquisition/DTScheduler.h
  src/SofaCV/acquisition/VideoGrabber.h
  
  src/SofaCV/datatypes/cvDMatch.h
  src/SofaCV/datatypes/cvKeypoint.h  
  src/SofaCV/datatypes/cvMat.h
  src/SofaCV/datatypes/cvMatUtils.h

  src/SofaCV/ICP/icp.h
  src/SofaCV/ICP/icpPointToPlane.h
  src/SofaCV/ICP/icpPointToPoint.h
  src/SofaCV/ICP/kdtree.h
  src/SofaCV/ICP/matrix.h

  src/SofaCV/utils/FrameExporter.h
  src/SofaCV/utils/FrameViewer.h
  src/SofaCV/utils/PCViewer.h

  src/SofaCV/DataSliderMgr.h
  src/SofaCV/DataSliderMgr.inl
  src/SofaCV/ImageFilter_oldGUI.h  
  )

set(SOURCE_FILES
  src/SofaCV/ImplicitDataEngine.cpp
  src/SofaCV/FemurConstraintForce.cpp
  src/SofaCV/DGConstraint.cpp

  src/SofaCV/acquisition/BaseFrameGrabber.cpp
  src/SofaCV/acquisition/BaseGrabber.cpp
  src/SofaCV/acquisition/BaseScheduler.cpp
  src/SofaCV/acquisition/DTScheduler.cpp
  src/SofaCV/acquisition/VideoGrabber.cpp 

  src/SofaCV/datatypes/cvDMatch.cpp
  src/SofaCV/datatypes/cvKeypoint.cpp
  src/SofaCV/datatypes/cvMat.cpp
  src/SofaCV/datatypes/cvMatUtils.cpp

  src/SofaCV/ICP/icp.cpp
  src/SofaCV/ICP/icpPointToPlane.cpp
  src/SofaCV/ICP/icpPointToPoint.cpp
  src/SofaCV/ICP/kdtree.cpp
  src/SofaCV/ICP/matrix.cpp

  src/SofaCV/utils/FrameExporter.cpp
  src/SofaCV/utils/FrameViewer.cpp
  src/SofaCV/utils/PCViewer.cpp

  src/SofaCV/ImageFilter_oldGUI.cpp
  )

add_library(${PROJECT_NAME} SHARED ${CONFIG_FILES} ${HEADER_FILES} ${SOURCE_FILES})
link_directories(${OpenCV_LIBRARY_DIRS})

target_link_libraries(${PROJECT_NAME} Sofa.Core)
target_link_libraries(${PROJECT_NAME} Sofa.Framework)
target_link_libraries(${PROJECT_NAME} Sofa.Simulation)
target_link_libraries(${PROJECT_NAME} Sofa.Component)
target_link_libraries(${PROJECT_NAME} Sofa.GL)
target_link_libraries(${PROJECT_NAME} Sofa.GUI)
target_link_libraries(${PROJECT_NAME} CImgPlugin)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} ${RealSense_LIBS})

target_compile_definitions(${PROJECT_NAME} PRIVATE "-DSOFA_BUILD_${PROJECT_NAME}")

sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION ${PROJECT_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    RELOCATABLE "plugins"
    )
