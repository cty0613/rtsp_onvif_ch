cmake_minimum_required(VERSION 3.10)
project(rtsp_onvif_ch VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
find_package(SDL2 REQUIRED)

# FFmpeg libraries
pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
    libavformat
    libavcodec
    libavutil
    libswscale
)

set(TFLITE_INCLUDE_DIR "/home/cty0613/tensorflow_src/bazel-tensorflow_src")         # 예: 빌드된 TFLite의 헤더 경로
set(TFLITE_STATIC_LIB "/home/cty0613/tensorflow_src/bazel-bin/tensorflow/lite/libtensorflowlite.so")  # 정적 라이브러리 경로
message(STATUS "Found TF-LITE Includes:" ${TFLITE_INCLUDE_DIR})
message(STATUS "Found TF-LITE Libs:" ${TFLITE_STATIC_LIB})

include_directories(${TFLITE_INCLUDE_DIR})

add_executable(rtsp_onvif_ch parser.cpp main.cpp video.cpp ocr.cpp)

# Include directories
target_include_directories(rtsp_onvif_ch PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${SDL2_INCLUDE_DIRS}
    ${TFLITE_INCLUDE_DIR}
)

# Link libraries
target_link_libraries(rtsp_onvif_ch PRIVATE 
    PkgConfig::LIBAV
    tinyxml2
    ${OpenCV_LIBS}
    ${SDL2_LIBRARIES}
    ${TFLITE_STATIC_LIB}
    rt
    pthread
)

# macOS specific settings
if(APPLE)
    target_link_libraries(rtsp_onvif_ch PRIVATE "-framework CoreFoundation" "-framework VideoToolbox" "-framework CoreVideo" "-framework CoreMedia")
endif()