cmake_minimum_required (VERSION 2.8)

set(CMAKE_MODULE_PATH "/usr/local/lib/cmake/${CMAKE_MODULE_PATH}")
find_package(raspicam REQUIRED)
find_package(OpenCV)
find_library(wiringPi_LIB wiringPi)


set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_executable (main main.cpp)

target_link_libraries (main ${raspicam_CV_LIBS})
target_link_libraries (main ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(main ${wiringPi_LIB})
