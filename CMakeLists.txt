cmake_minimum_required(VERSION 3.0)
project(Libcbdetect)

# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE})
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib/${CMAKE_BUILD_TYPE})
# set(CMAKE_LIBRARY_PATH ${PROJECT_SOURCE_DIR}/lib/${CMAKE_BUILD_TYPE})

set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

if (DEFINED ENV{CONDA_PREFIX})
    set(CMAKE_PREFIX_PATH $ENV{CONDA_PREFIX})
endif()

set(LIB_TYPE STATIC)

# Set the path to the Homebrew-installed OpenCV libraries
# set(OpenCV_DIR "/usr/local/opt/opencv@4")

# find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
find_package(OpenCV REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

# add_subdirectory(path/to/cvnp)
# target_link_libraries(your_target PRIVATE cvnp)

set(THIRD_PARTY_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})
set(THIRD_PARTY_LIBS ${OpenCV_LIBS})
include_directories(${THIRD_PARTY_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include)

add_subdirectory(${PROJECT_SOURCE_DIR}/src)
add_subdirectory(${PROJECT_SOURCE_DIR}/src/libcbdetect)

pybind11_add_module(cbdetect_py MODULE src/python/libcbdetect.cc)
target_link_libraries(cbdetect_py PRIVATE cbdetect)

install(TARGETS cbdetect_py DESTINATION ${SKBUILD_PROJECT_NAME})
