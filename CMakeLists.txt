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

add_executable(rtsp_onvif_ch parser.cpp main.cpp video.cpp)

# Include directories
target_include_directories(rtsp_onvif_ch PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${SDL2_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(rtsp_onvif_ch PRIVATE 
    PkgConfig::LIBAV
    tinyxml2
    ${OpenCV_LIBS}
    ${SDL2_LIBRARIES}
    rt
    pthread
)

# macOS specific settings
if(APPLE)
    target_link_libraries(rtsp_onvif_ch PRIVATE "-framework CoreFoundation" "-framework VideoToolbox" "-framework CoreVideo" "-framework CoreMedia")
endif()