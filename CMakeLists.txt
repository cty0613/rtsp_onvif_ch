cmake_minimum_required(VERSION 3.10)
project(rtsp_onvif_ch VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# FFmpeg
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
    libavformat
    libavcodec
    libavutil
    libswscale
)

# TinyXML2
find_package(tinyxml2 REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

add_executable(rtsp_onvif_ch parser.cpp main.cpp video.cpp)

# include paths
target_include_directories(rtsp_onvif_ch PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

# link libraries
target_link_libraries(rtsp_onvif_ch PRIVATE 
    PkgConfig::LIBAV
    tinyxml2::tinyxml2
    ${OpenCV_LIBS}
)

# macOS specific settings
if(APPLE)
    target_link_libraries(rtsp_onvif_ch PRIVATE "-framework CoreFoundation" "-framework VideoToolbox" "-framework CoreVideo" "-framework CoreMedia")
endif()

# install rules
install(TARGETS rtsp_onvif_ch DESTINATION bin)