cmake_minimum_required(VERSION 3.0)
project(sfm)

# set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

SET(OpenCV_INCLUDE_DIRS /usr/local/include/opencv4/opencv2)
SET(OpenCV_DIR /usr/local/lib/cmake/opencv4/OpenCVConfig.cmake)
SET(OpenCV_LIBRARIES /usr/local/lib)


find_package(OpenCV 4.4.0 REQUIRED)

# include_directories(
#     ${PROJECT_SOURCE_DIR}/include
#     ${PROJECT_SOURCE_DIR}/src
# )
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(/usr/local/lib)
# # file(GLOB SOURCES  *.cpp *hpp)
# file(GLOB all_SRCS
#     "${PROJECT_SOURCE_DIR}/include/*.hpp"
#     "${PROJECT_SOURCE_DIR}/include/*.cpp"
# )

# link_directories(${PROJECT_SOURCE_DIR}/include)

add_library(frame_list include/frame_list.hpp)
target_link_libraries(frame_list ${OpenCV_LIBRARIES})
set_target_properties(frame_list PROPERTIES LINKER_LANGUAGE CXX)


add_library(relative_pose include/relative_pose.hpp)
target_link_libraries(relative_pose ${OpenCV_LIBRARIES})
set_target_properties(relative_pose PROPERTIES LINKER_LANGUAGE CXX)


add_executable(main src/main.cpp)
# add_library(frame_list include/frame_list.hpp)
target_link_libraries(main ${OpenCV_LIBRARIES} frame_list relative_pose)


# add_executable(foo src/foo.cpp)
# add_library(moo src/moo.cpp)
# target_link_libraries(foo moo) //foo라는 실행파일 타겟과 링크될 라이브러리를 지정.