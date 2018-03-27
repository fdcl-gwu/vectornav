cmake_minimum_required(VERSION 2.8.4)
project(vn100_test)

set(CMAKE_SUPPRESS_REGENERATION TRUE)

add_subdirectory(../c libvnc)
add_subdirectory(../cpp libvncxx)
#add_subdirectory(../c/examples/getting_started getting_started)

if(CMAKE_COMPILER_ID MATCHES "GNU|Clang")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

set(SOURCE_FILES
    main.cpp)

include_directories(../c/include)
include_directories(../cpp/include)

add_executable(vn100_test ${SOURCE_FILES})

target_link_libraries(vn100_test PRIVATE libvnc libvncxx)

if (UNIX OR APPLE)
	target_link_libraries(vn100_test LINK_PUBLIC pthread)
else()
	target_link_libraries(vn100_test LINK_PUBLIC Advapi32 SetupAPI)
endif()
