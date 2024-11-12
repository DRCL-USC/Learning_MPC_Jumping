cmake_minimum_required(VERSION 2.8.3)

project(libuscbot)
include_directories(".")
include_directories(include)
include_directories(src)
include_directories(sdk/include)
link_directories(sdk/lib)
include_directories(${CMAKE_BINARY_DIR})
include_directories(../third-party/)
include_directories(include/NatNet)

file(GLOB_RECURSE sources "src/*.cpp" "*.h" ".hpp")

add_compile_options(-std=c++11)

set(EXTRA_LIBS -pthread libunitree_legged_sdk.so lcm libNatNet.so)
set(CMAKE_CXX_FLAGS "-O3 -ggdb -no-pie")

add_library(uscbot SHARED ${sources})
target_link_libraries(uscbot ${EXTRA_LIBS})

add_executable(leg_test test/LegControlTest.cpp)
target_link_libraries(leg_test uscbot ${EXTRA_LIBS})

add_executable(imu_test test/imuTest.cpp)
target_link_libraries(imu_test uscbot ${EXTRA_LIBS})

add_executable(swing_test test/footSwingTest.cpp)
target_link_libraries(swing_test uscbot ${EXTRA_LIBS})

add_executable(gamepad_test test/gamepadTest.cpp)
target_link_libraries(gamepad_test uscbot ${EXTRA_LIBS})
