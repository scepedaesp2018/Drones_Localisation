cmake_minimum_required(VERSION 2.8.3)
project(Drons_detection)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug)

find_package( OpenCV REQUIRED )

# subdirectory
# add_subdirectory(img)

add_executable(detection_video src/video.cpp)
add_executable(detection_photo src/images.cpp)

target_link_libraries(detection_video ${OpenCV_LIBS} )
target_link_libraries(detection_photo ${OpenCV_LIBS} )

# target_include_directories(detection PRIVATE ${../img})

# include_directories(../img)
