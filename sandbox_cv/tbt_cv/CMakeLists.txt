cmake_minimum_required(VERSION 3.8)
project(Tennis_Ball_Detector)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_AUTOMOC ON)

# find the relevant/required CMake libraries
find_package(OpenCV REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

# include OpenCV directories
include_directories(${OpenCV_INCLUDE_DIRS})

# create the executable
add_executable(
    ${PROJECT_NAME}_image
    tennis_ball_detector_image_main.cpp
    tennis_ball_detector.h
    tennis_ball_detector.cpp
)

# link the appropriate OpenCV libraries
target_link_libraries( ${PROJECT_NAME}_image
   ${OpenCV_LIBS}
   Qt5::Core
   Qt5::Widgets
)

# create the executable
add_executable(
    ${PROJECT_NAME}_video
    tennis_ball_detector_video_main.cpp
    tennis_ball_detector.h
    tennis_ball_detector.cpp
)

# link the appropriate OpenCV libraries
target_link_libraries( ${PROJECT_NAME}_video
   ${OpenCV_LIBS}
   Qt5::Core
   Qt5::Widgets
)
