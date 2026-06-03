# CMakeLists.txt
cmake_minimum_required(VERSION 3.31)

project(controller)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Fetch protobuf and Abseil library:
include(FetchContent)

# Abseil library (required by protobuf)
set(ABSL_PROPAGATE_CXX_STD ON CACHE BOOL "" FORCE)
set(ABSL_MSVC_STATIC_RUNTIME OFF CACHE BOOL "" FORCE)
FetchContent_Declare(absl
  URL https://github.com/abseil/abseil-cpp/archive/refs/tags/20240722.0.zip
  URL_HASH MD5=4da741f7bcdd353c441ecdc2f7fc5716
  SYSTEM EXCLUDE_FROM_ALL
)
FetchContent_MakeAvailable(absl)

# Protobuf library
set(protobuf_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(protobuf_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(protobuf_MSVC_STATIC_RUNTIME OFF CACHE BOOL "" FORCE)
set(protobuf_WITH_ZLIB   OFF CACHE BOOL "" FORCE)
set(protobuf_INSTALL     OFF CACHE BOOL "" FORCE)
set(protobuf_VERBOSE    ON CACHE BOOL "" FORCE)
set(protobuf_BUILD_LIBUPB OFF CACHE BOOL "" FORCE)
FetchContent_Declare(protobuf
  URL https://github.com/protocolbuffers/protobuf/releases/download/v28.0/protobuf-28.0.zip
  URL_HASH MD5=805d4146eb1a787ce49642cc4d911256
  SYSTEM EXCLUDE_FROM_ALL
)
FetchContent_MakeAvailable(protobuf)
include(${protobuf_SOURCE_DIR}/cmake/protobuf-generate.cmake)

# The Fuzzy RTOS protobufs
add_library(${PROJECT_NAME}_protos STATIC
    protos/api_v2.proto
    protos/api_v2_rtos.proto
)
protobuf_generate(
  TARGET ${PROJECT_NAME}_protos
  LANGUAGE cpp
  IMPORT_DIRS ${protobuf_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(${PROJECT_NAME}_protos PUBLIC protobuf::libprotobuf)
target_include_directories(${PROJECT_NAME}_protos PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

#===============Source files and headers==================

set(SRCS
src/main.cpp
src/controller.cpp
src/socket.cpp
)

set (HEADERS
include/socket.hpp
include/controller.hpp
)
#===================Main executable==========================
add_executable(main ${SRCS} ${HEADERS})


#==========================================================
target_include_directories(main PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(main PRIVATE ${PROJECT_NAME}_protos)