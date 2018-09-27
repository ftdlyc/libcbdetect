include_directories(${PROJECT_SOURCE_DIR}/include/libcbdetect)
aux_source_directory(. SOURCE_LIST)
add_library(cbdetect SHARED ${SOURCE_LIST})
