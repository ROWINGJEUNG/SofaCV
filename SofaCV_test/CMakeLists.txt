cmake_minimum_required(VERSION 3.1)

project(SofaCV_test)

find_package(OpenCV REQUIRED)
find_package(SofaCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

set(HEADER_FILES
  )

set(SOURCE_FILES
  ImplicitDataEngine_test.cpp
  BaseFrameGrabber_test.cpp
  )

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} SofaTest SofaGTestMain SofaCV ${OpenCV_LIBRARIES})

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
