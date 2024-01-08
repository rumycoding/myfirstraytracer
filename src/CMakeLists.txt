cmake_minimum_required(VERSION 3.10)
project(RayTracing)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CAMKE_CXX_FLAGS} -fopenmp")

add_executable(RayTracing main.cpp Renderer.cpp Scene.cpp BVH.cpp PhotonMap.cpp)

# the following are for testing
find_package(Catch2 3 REQUIRED)
add_executable(tests Test.cpp Renderer.cpp Scene.cpp BVH.cpp PhotonMap.cpp)
target_link_libraries(tests PRIVATE Catch2::Catch2WithMain)

include(CTest)
include(Catch)
catch_discover_tests(tests)