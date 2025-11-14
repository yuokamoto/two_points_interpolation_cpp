cmake_minimum_required(VERSION 3.12)
project(TwoPointsInterpolationExamples)

set(CMAKE_CXX_STANDARD 11)

# Find yaml-cpp library
find_package(yaml-cpp REQUIRED)

# Include parent directory for headers
include_directories(${CMAKE_SOURCE_DIR}/../include)

# Add the constant acceleration executable
add_executable(acc_example constant_acc_example.cpp)
target_include_directories(acc_example PRIVATE ${YAML_CPP_INCLUDE_DIR})
target_link_libraries(acc_example PRIVATE ${YAML_CPP_LIBRARIES})

# Add the constant jerk executable  
add_executable(jerk_example constant_jerk_example.cpp)
target_include_directories(jerk_example PRIVATE ${YAML_CPP_INCLUDE_DIR})
target_link_libraries(jerk_example PRIVATE ${YAML_CPP_LIBRARIES})