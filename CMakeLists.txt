cmake_minimum_required(VERSION 3.15)
project(
  ${SKBUILD_PROJECT_NAME}
  LANGUAGES CXX
  VERSION ${SKBUILD_PROJECT_VERSION}
  )

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE})
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib/${CMAKE_BUILD_TYPE})
set(CMAKE_LIBRARY_PATH ${PROJECT_SOURCE_DIR}/lib/${CMAKE_BUILD_TYPE})

set(CMAKE_CXX_STANDARD 14)

# if (DEFINED ENV{CONDA_PREFIX})
#     set(CMAKE_PREFIX_PATH $ENV{CONDA_PREFIX})
# endif()
find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)

find_package(OpenCV REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

set(THIRD_PARTY_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})
set(THIRD_PARTY_LIBS ${OpenCV_LIBS})
include_directories(${THIRD_PARTY_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include)

add_subdirectory(${PROJECT_SOURCE_DIR}/src)
add_subdirectory(${PROJECT_SOURCE_DIR}/src/libcbdetect)

pybind11_add_module(cbdetect_py MODULE src/python/libcbdetect.cc)
target_link_libraries(cbdetect_py PRIVATE cbdetect)
target_compile_definitions(cbdetect_py PRIVATE VERSION_INFO=${PROJECT_VERSION})

if(DEFINED SKBUILD_PROJECT_NAME)
  install(TARGETS cbdetect_py LIBRARY DESTINATION .)
endif()
