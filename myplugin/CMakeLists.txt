cmake_minimum_required (VERSION 2.6.0)
project(myplugin)
find_package(OpenRAVE 0.8 REQUIRED)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

# optional in case boost is used
find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT)

## Set build flags for working with OpenMP
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -O3")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${OpenMP_SHARED_LINKER_FLAGS}")

include_directories(${OpenRAVE_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_library(myplugin SHARED myplugin.cpp)
set_target_properties(myplugin PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(myplugin ${OpenRAVE_LIBRARIES})
install(TARGETS myplugin DESTINATION .)
