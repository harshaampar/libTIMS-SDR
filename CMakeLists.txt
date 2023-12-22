cmake_minimum_required(VERSION 3.22)
project(TIMSSDR C)

set(c_sources ${CMAKE_CURRENT_SOURCE_DIR}/src/timssdr.c CACHE INTERNAL "List of C sources")
set(c_headers ${CMAKE_CURRENT_SOURCE_DIR}/include/timssdr.h CACHE INTERNAL "List of C headers")

add_library(timssdr SHARED ${c_sources})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
set_target_properties(timssdr PROPERTIES CLEAN_DIRECT_OUTPUT 1)
target_link_libraries(timssdr usb  usb-1.0 pthread)

add_library(timssdr_static STATIC ${c_sources})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
set_target_properties(timssdr_static PROPERTIES CLEAN_DIRECT_OUTPUT 1)
target_link_libraries(timssdr_static usb usb-1.0 pthread)

add_executable(timssdr-info ${CMAKE_CURRENT_SOURCE_DIR}/tests/timssdr-info.c)
target_include_directories(timssdr-info PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(timssdr-info timssdr)

install(TARGETS timssdr
LIBRARY DESTINATION /usr/lib/x86_64-linux-gnu/
COMPONENT sharedlibs
)

install(FILES ${c_headers}
DESTINATION /usr/include/
COMPONENT headers
)
