cmake_minimum_required(VERSION 3.12)
project(TwoPointsInterpolationExample)

set(CMAKE_CXX_STANDARD 11)

# Find yaml-cpp library
find_package(yaml-cpp REQUIRED)

# Add the executable target
add_executable(${PROJECT_NAME} two_points_interpolation_constant_acc_example.cpp)

# Include the directories for yaml-cpp
target_include_directories(${PROJECT_NAME} PRIVATE ${YAML_CPP_INCLUDE_DIR})

# Link against yaml-cpp library
target_link_libraries(${PROJECT_NAME} PRIVATE ${YAML_CPP_LIBRARIES})