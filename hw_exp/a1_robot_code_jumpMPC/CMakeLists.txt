cmake_minimum_required(VERSION 2.8.3)
project(a1_robot_build)

message("==== for a1 robot build ====")

set(CMAKE_CXX_FLAGS "-O3 -no-pie -ggdb -Wall \
-Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
-Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
-Wshadow -Wsign-promo -Werror")
set(CMAKE_C_FLAGS "-O3 -ggdb -std=gnu99 -I -fPIC")
set(CMAKE_CXX_STANDARD 14)

add_subdirectory(low_level_controller)
add_subdirectory(third-party)
add_subdirectory(Controllers)
