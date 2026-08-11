cmake_minimum_required(VERSION 3.20)
project(modelling_control_and_simulation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Tell CMake where to find Drake.
# Point this to the absolute or relative path to drake-build
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_LIST_DIR}/../drake-build/install")

find_package(drake REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} 
    SHARED
        src/controllers/acrobot/collocated_energy_ctrl.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC 
    drake::drake
    yaml-cpp::yaml-cpp
)

add_executable(acrobot_collocated_energy_ctrl apps/acrobot_collocated_energy_ctrl.cpp)
add_executable(acrobot_visualization apps/acrobot_visualization.cpp)

target_link_libraries(acrobot_collocated_energy_ctrl
    PUBLIC
        ${PROJECT_NAME}
)

target_link_libraries(acrobot_visualization
    PUBLIC
        ${PROJECT_NAME}
)

