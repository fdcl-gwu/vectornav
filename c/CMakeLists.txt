cmake_minimum_required(VERSION 2.8.4)
project(libvnc C)

#set(CMAKE_SUPPRESS_REGENERATION TRUE)

include_directories(
    include
    )

set(SOURCES
        src/vn/xplat/criticalsection.c
        src/vn/xplat/event.c
        src/vn/xplat/serialport.c
        src/vn/xplat/thread.c
        src/vn/xplat/time.c
        src/vn/error.c
        src/vn/error_detection.c
        src/vn/util.c
        src/vn/math/matrix.c
        src/vn/math/vector.c
        src/vn/protocol/upack.c
        src/vn/protocol/upackf.c
        src/vn/protocol/spi.c
        src/vn/sensors/compositedata.c
        src/vn/sensors/searcher.c
        src/vn/sensors.c
        src/vn/sensors/ezasyncdata.c
        src/vn/conv.c
        )

add_library(libvnc ${SOURCES})
