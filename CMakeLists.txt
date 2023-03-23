cmake_minimum_required(VERSION 2.8.12)
project(SofaCV)

set(${PROJECT_NAME}_MAJOR_VERSION 18)
set(${PROJECT_NAME}_MINOR_VERSION 12)
set(${PROJECT_NAME}_VERSION ${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION})

find_package(OpenCV REQUIRED)
find_package(SofaFramework REQUIRED)
find_package(SofaSimulation REQUIRED)
find_package(SofaBase REQUIRED)
find_package(Sofa.GL REQUIRED)
find_package(SofaQtQuickGUI QUIET)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

add_definitions("-std=c++11")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(CONFIG_FILES
  src/SofaCV/SofaCVPlugin.h
  src/SofaCV/SofaCVPlugin.cpp
  )

set(HEADER_FILES
  src/SofaCV/SofaCV.h
  src/SofaCV/datatypes/cvKeypoint.h
  src/SofaCV/datatypes/cvDMatch.h
  src/SofaCV/datatypes/cvMat.h
  src/SofaCV/datatypes/cvMatUtils.h
  src/SofaCV/utils/FrameViewer.h
  src/SofaCV/utils/FrameExporter.h
  src/SofaCV/utils/PCViewer.h
  src/SofaCV/acquisition/BaseScheduler.h
  src/SofaCV/acquisition/DTScheduler.h
  src/SofaCV/acquisition/BaseGrabber.h
  src/SofaCV/acquisition/BaseFrameGrabber.h
  src/SofaCV/acquisition/VideoGrabber.h
  src/SofaCV/ImplicitDataEngine.h
  src/SofaCV/ImageFilter.h
  )

set(SOURCE_FILES
  src/SofaCV/datatypes/cvKeypoint.cpp
  src/SofaCV/datatypes/cvDMatch.cpp
  src/SofaCV/datatypes/cvMat.cpp
  src/SofaCV/datatypes/cvMatUtils.cpp
  src/SofaCV/utils/FrameViewer.cpp
  src/SofaCV/utils/FrameExporter.cpp
  src/SofaCV/utils/PCViewer.cpp
  src/SofaCV/acquisition/BaseScheduler.cpp
  src/SofaCV/acquisition/DTScheduler.cpp
  src/SofaCV/acquisition/BaseGrabber.cpp
  src/SofaCV/acquisition/BaseFrameGrabber.cpp
  src/SofaCV/acquisition/VideoGrabber.cpp
  src/SofaCV/ImplicitDataEngine.cpp
  )

find_package(SofaPython QUIET)
if( SofaPython_FOUND )
  set(PYTHON_FILES
    src/SofaCV/python/SofaCV/__init__.py
    )

  list(APPEND SOURCE_FILES src/SofaCV/python/Binding_cvMatData.cpp)
  list(APPEND HEADER_FILES
    src/SofaCV/python/Binding_cvMatData.h)
  sofa_set_python_directory(${PROJECT_NAME} "src/SofaCV/python")
endif()
find_package(SofaPython3 QUIET)

set(${PROJECT_NAME}_FILES
  ${CONFIG_FILES}
  ${HEADER_FILES}
  ${SOURCE_FILES}
  )

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(${PROJECT_NAME}_LIBRARIES
  SofaCore
  SofaSimulationCommon
  SofaBaseVisual
  Sofa.GL
  ${OpenCV_LIBRARIES}
  )

set (SOFACV_NEW_GUI_ENABLED OFF CACHE BOOL "Enables ImageFilter Widgets for runSofa2")
if(SofaQtQuickGUI_FOUND AND ${SOFACV_NEW_GUI_ENABLED})
    message ("WE FOUND THE NEW GUI! COOL!")
  find_package(Qt5QuickControls2 REQUIRED)
  find_package(Qt5Quick REQUIRED)
  find_package(Qt5Qml REQUIRED)
  find_package(GLEW REQUIRED)

  add_definitions("-DSOFACV_NEW_GUI_ENABLED")
  add_definitions("-DSOFA_BUILD_${PROJECT_NAME}_PLUGIN")
  add_definitions("-DHAS_GLEW")

  list(APPEND SOURCE_FILES
    src/SofaCV/gui/ImageFilterDisplay.cpp
    src/SofaCV/gui/ImageFilterModel.cpp
    src/SofaCV/ImageFilter_newGUI.cpp
    )

  list(APPEND HEADER_FILES
    src/SofaCV/gui/ImageFilterDisplay.h
    src/SofaCV/gui/ImageFilterModel.h
    src/SofaCV/ImageFilter_newGUI.h
    )

    set(MOC_HEADER_FILES
    src/SofaCV/gui/ImageFilterDisplay.h
    src/SofaCV/gui/ImageFilterModel.h
    )

  set(QML_FILES
    src/SofaCV/gui/qml/SofaDataTypes/SofaDataType_cvMat.qml
    src/SofaCV/gui/qml/SofaDataTypes/SofaDataType_cvDMatch.qml
    src/SofaCV/gui/qml/SofaDataTypes/SofaDataType_cvKeypoint.qml
    src/SofaCV/gui/qml/SofaWidgets/ImageFilterWidget.qml
    )

  set(QRC_FILES
    src/SofaCV/gui/qml/SofaCV_qml.qrc
    )

  list (APPEND ${PROJECT_NAME}_LIBRARIES
    SofaQtQuickGUI
    ${GLEW_LIBRARIES}
    )
  # do not use "set(CMAKE_AUTOMOC ON)" since all the mocced files
  # will be compiled by a single compilation unit leading to a
  # "out of heap space" issue on MSVC
  qt5_wrap_cpp(MOC_FILES ${MOC_HEADER_FILES})
  qt5_add_resources(RESOURCE_FILES ${QRC_FILES})

  list (APPEND ${PROJECT_NAME}_FILES
    ${CONFIG_FILES}  
    ${SOURCE_FILES}
    ${HEADER_FILES}
    ${MOC_FILES}
    ${RESOURCE_FILES}
    ${QML_FILES}
    ${QRC_FILES}
    )
  if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DQT_NO_DEBUG)
  endif(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_definitions(-DQT_PLUGIN)
else ()
  list(APPEND HEADER_FILES
    src/SofaCV/DataSliderMgr.h
    src/SofaCV/DataSliderMgr.inl
    src/SofaCV/ImageFilter_oldGUI.h
    )

  list(APPEND SOURCE_FILES
    src/SofaCV/ImageFilter_oldGUI.cpp
    )
  set(${PROJECT_NAME}_FILES
    ${CONFIG_FILES}
    ${SOURCE_FILES}
    ${HEADER_FILES}
    )
endif (SofaQtQuickGUI_FOUND AND ${SOFACV_NEW_GUI_ENABLED})

set(${PROJECT_NAME}_RESOURCES_PATH \"${CMAKE_CURRENT_SOURCE_DIR}/resources\")
configure_file("src/SofaCV/${PROJECT_NAME}.h.in" "${CMAKE_CURRENT_SOURCE_DIR}/src/SofaCV/${PROJECT_NAME}.h")
configure_file("src/SofaCV/${PROJECT_NAME}.h.in" "${CMAKE_CURRENT_BINARY_DIR}/include/SofaCV/${PROJECT_NAME}.h")
set(${PROJECT_NAME}_RESOURCES_PATH  \"${CMAKE_INSTALL_PREFIX}/share/sofa/plugins/${PROJECT_NAME}\")
configure_file("src/SofaCV/${PROJECT_NAME}.h.in" "${CMAKE_CURRENT_BINARY_DIR}/include/SofaCV/installed${PROJECT_NAME}.h")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/include/SofaCV/installed${PROJECT_NAME}.h" DESTINATION "include/${PROJECT_NAME}/" RENAME "${PROJECT_NAME}.h")

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_FILES} ${PYTHON_FILES})

if ( SofaPython_FOUND )
  target_link_libraries(${PROJECT_NAME} PRIVATE SofaPython)
  add_definitions("-DSOFA_HAVE_SOFAPYTHON")
endif( SofaPython_FOUND )

target_link_libraries(${PROJECT_NAME} PRIVATE ${${PROJECT_NAME}_LIBRARIES})

target_compile_options(${PROJECT_NAME} PUBLIC -D${PROJECT_NAME}_VERSION=\"${${PROJECT_NAME}_VERSION}\")

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}")


sofa_generate_package(
  NAME ${PROJECT_NAME}
  VERSION ${${PROJECT_NAME}_VERSION}
  TARGETS ${PROJECT_NAME}
  INCLUDE_INSTALL_DIR ${PROJECT_NAME}
  INCLUDE_SOURCE_DIR "src/${PROJECT_NAME}"
  RELOCATABLE "plugins"
  )

install(DIRECTORY resources/ DESTINATION share/sofa/plugins/${PROJECT_NAME})


if( SofaPython3_FOUND )
  add_subdirectory(python3)
endif()

if(SOFA_BUILD_TESTS)
    find_package(SofaTest QUIET)
    if(SofaTest_FOUND)
        add_subdirectory(SofaCV_test)
    endif(SofaTest_FOUND)
endif(SOFA_BUILD_TESTS)
