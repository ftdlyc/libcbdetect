cmake_minimum_required(VERSION 2.8)
project(Libcbdetect)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE})
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib/${CMAKE_BUILD_TYPE})
set(CMAKE_LIBRARY_PATH ${PROJECT_SOURCE_DIR}/lib/${CMAKE_BUILD_TYPE})

if (UNIX)
  set(CMAKE_CXX_COMPILER g++)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -fopenmp")
  add_definitions(-DUNIX)
elseif (WIN32)
  add_definitions(-DWINDOWS)
  if (MSVC)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -openmp")
    add_definitions("/EHsc")
  endif ()
endif ()

option(SHARED_LIB "Request build of shared libraries." ON)
if (SHARED_LIB)
  if (WIN32)
    add_definitions("-DIS_A_DLL")
  endif ()
  set(LIB_TYPE SHARED)
else ()
  set(LIB_TYPE STATIC)
endif ()

find_package(OpenCV REQUIRED)

set(THIRD_PARTY_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})
set(THIRD_PARTY_LIBS ${OpenCV_LIBS})
include_directories(${THIRD_PARTY_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include)

add_subdirectory(${PROJECT_SOURCE_DIR}/src)
add_subdirectory(${PROJECT_SOURCE_DIR}/src/libcbdetect)
