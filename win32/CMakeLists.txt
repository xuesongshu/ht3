cmake_minimum_required(VERSION 3.13)
project(test-fx3)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_GENERATOR_PLATFORM x64)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/build)
set(LIBRARY_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})

set(FX3 "C:/Program Files (x86)/Cypress/EZ-USB FX3 SDK/1.3")
include_directories("${FX3}/library/cpp/inc")
link_directories("${FX3}/library/cpp/lib/x64")
link_libraries(legacy_stdio_definitions SetupAPI CyAPI)
add_compile_definitions(UNICODE _DEBUG)
add_link_options(/NODEFAULTLIB:libcmt)

file(GLOB SRCS *.cpp *.c)
add_executable(${PROJECT_NAME} ${SRCS})