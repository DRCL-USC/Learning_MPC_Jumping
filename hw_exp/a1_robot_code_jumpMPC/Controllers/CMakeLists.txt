cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)

list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/Downloads/libtorch-cxx11--2.0.1/libtorch")
find_package(Torch REQUIRED)
find_package(qpOASES REQUIRED)

include_directories("./")
include_directories(../low_level_controller)
include_directories(../low_level_controller/include)
#include_directories("../third-party/qpOASES/include/")
include_directories("../third-party/JCQP")
include_directories("ConvexMPC/")
include_directories("BalanceController/")
include_directories(../low_level_controller/sdk/include)
link_directories(../low_level_controller/sdk/lib)
include_directories("/usr/local/include/eigen3") 
include_directories(${CMAKE_BINARY_DIR})
include_directories(${qpOASES_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "-O3 -std=gnu99 -I")
set(CMAKE_CXX_FLAGS "-O3 -ggdb -no-pie")
set(EXTRA_LIBS -pthread libunitree_legged_sdk.so lcm)


file(GLOB_RECURSE sources "*.cpp" "*.h" ".hpp")

add_executable(ctrl ${sources})
target_link_libraries(ctrl uscbot ${EXTRA_LIBS} qpOASES JCQP ${TORCH_LIBRARIES}
	/opt/intel/mkl/lib/intel64/libmkl_intel_thread.so
	/opt/intel/mkl/lib/intel64/libmkl_core.so
	/opt/intel/mkl/lib/intel64/libmkl_intel_ilp64.so)

#target_link_libraries(ctrl uscbot ${EXTRA_LIBS} ${qpOASES_LIB} JCQP)
