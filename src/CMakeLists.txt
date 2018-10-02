SET(pwd ${CMAKE_CURRENT_LIST_DIR})

SET(src "")
AUX_SOURCE_DIRECTORY(${pwd} src)
add_executable(pdf ${src})
message(STATUS "pdf added ${src}")

target_link_libraries(pdf ${stdgl_libraries})
FIND_PACKAGE(JPEG REQUIRED)
TARGET_LINK_LIBRARIES(pdf ${JPEG_LIBRARIES})
TARGET_LINK_LIBRARIES(pdf ${OPENCV_LIBRARIES})
TARGET_LINK_LIBRARIES(pdf pmdreader)
