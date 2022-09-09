

find_package(OpenCV 4.1.1 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})
message("opencv_DIR:" ${OpenCV_DIR})
message("OpenCV_LIBRARY_DIRS:" ${OpenCV_LIBRARY_DIRS} )
message("OpenCV_INCLUDE_DIRS:" ${OpenCV_INCLUDE_DIRS} )

list(APPEND ALL_TARGET_LIBRARIES ${OpenCV_LIBRARIES})