cmake_minimum_required(VERSION 3.16.3)

project(crane)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_crane)

add_executable(capture examples/capture.cpp)
target_link_libraries(capture ${OpenCV_LIBS} io fmt::fmt tools yaml-cpp)

add_executable(cboard_test examples/cboard_test.cpp)
target_link_libraries(cboard_test fmt::fmt io tools)

add_executable(cboard_tx_test examples/cboard_tx_test.cpp)
target_link_libraries(cboard_tx_test fmt::fmt io tools)

add_executable(usbcamera_test examples/usbcamera_test.cpp)
target_link_libraries(usbcamera_test ${OpenCV_LIBS} io fmt::fmt tools)

# add_executable(localizer_test examples/localizer_test.cpp)
# target_link_libraries(localizer_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_crane tools io)

add_executable(matcher_test examples/matcher_test.cpp)
target_link_libraries(matcher_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_crane tools io)

add_executable(t_cam2gripper_test examples/t_cam2gripper_test.cpp)
target_link_libraries(t_cam2gripper_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_crane tools io)

# add_executable(crane src/crane.cpp)
# target_link_libraries(crane ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_crane tools io)
