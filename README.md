# RTSP ONVIF Object Detection Viewer

A real-time program that parses ONVIF metadata from RTSP streams and displays object detection results as overlays on video. specially for Hanwha IP Camera RTSP Stream. 

## Key Features

- Simultaneous reception of video and metadata from RTSP streams
- ONVIF XML metadata parsing (object detection information)
- Real-time object detection box overlay display
- Support for various object types (Head, Human, Face, etc.)
- Object information display (type, ID, confidence, center point)

## Dependencies

### System Requirements
- C++17 compatible compiler
- CMake 3.10 or higher
- pkg-config

### Library Dependencies

#### FFmpeg
Video stream decoding and processing
```bash
# macOS (Homebrew)
brew install ffmpeg

# Ubuntu/Debian
sudo apt update
sudo apt install libavformat-dev libavcodec-dev libavutil-dev libswscale-dev

# CentOS/RHEL
sudo yum install ffmpeg-devel
```

#### OpenCV
Video display and graphics overlay
```bash
# macOS (Homebrew)
brew install opencv

# Ubuntu/Debian
sudo apt install libopencv-dev

# CentOS/RHEL
sudo yum install opencv-devel
```

#### TinyXML2
XML metadata parsing
```bash
# macOS (Homebrew)
brew install tinyxml2

# Ubuntu/Debian
sudo apt install libtinyxml2-dev

# CentOS/RHEL
sudo yum install tinyxml2-devel
```

## Build Instructions

### 1. Clone Repository
```bash
git clone <repository-url>
cd rtsp_onvif_ch
```

### 2. Create Build Directory and Build
```bash
mkdir build
cd build
cmake ..
make
```

### 3. Run
```bash
./rtsp_onvif_ch
```

## Project Structure

```
rtsp_onvif_ch/
├── CMakeLists.txt          # CMake build configuration
├── main.cpp                # Main program entry point
├── parser.hpp/cpp          # ONVIF XML metadata parser
├── video.hpp/cpp           # Video processing and overlay rendering
├── build/                  # Build output directory
└── README.md              # Project documentation
```


## Configuration

To modify the currently hardcoded RTSP URL, edit the following line in `main.cpp`:
```cpp
const char* url = "rtsp://192.168.0.64/profile2/media.smp";
```

