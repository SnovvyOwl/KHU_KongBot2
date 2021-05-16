cmake_minimum_required (VERSION 2.8) 
project(KF_test)
set(CMAKE_MODULE_PATH "/usr/local/lib/cmake/${CMAKE_MODULE_PATH}") 

include_directories(${PROJECT_SOURCE_DIR})
set(SOURCES KF_Test_SW.cpp )
find_package(OpenCV)
include_directories( ${OpenCV_INCLUDE_DIRS} )
IF (OpenCV_FOUND)
MESSAGE(STATUS "KF_test code COMPILING..")
endif()


add_executable (KF_test ${SOURCES})
target_link_libraries(KF_test ${OpenCV_LIBS} )