#ifndef VIDEO_HPP
#define VIDEO_HPP

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/frame.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
#include <libavutil/avutil.h>
}

#include <opencv2/opencv.hpp>
#include <memory>
#include <queue>
#include <deque>
#include "parser.hpp"

class VideoProcessor {
private:
    AVCodecContext* codec_context_;
    const AVCodec* codec_;
    SwsContext* sws_context_;
    AVFrame* frame_;
    AVFrame* frame_rgb_;
    uint8_t* buffer_;
    int width_;
    int height_;
    bool initialized_;
    
    // OpenCV window name
    std::string window_name_;
    
    // Detection results to draw
    std::deque< std::vector<Object> > detection_objects_queue_;
    std::mutex objects_mutex_;
    
    // DTS info
    std::deque<int64_t> dts_queue_;
    
public:
    VideoProcessor(const std::string& window_name = "RTSP Video Stream");
    ~VideoProcessor();
    
    bool initialize(AVCodecParameters* codecpar);
    
    bool processPacket(AVPacket* packet, int64_t dts);
    
    bool isInitialized() const { return initialized_; }
    
    void setDetectionResults(const std::vector<Object>& objects, const int64_t dts);
    
    // Get video dimensions
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    
    // Cleanup resources
    void cleanup();
    
private:
    // Convert AVFrame to OpenCV Mat and display with detection boxes
    void displayFrame(AVFrame* frame);
    
    // Draw detection boxes
    void drawDetectionBoxes(cv::Mat& image);
};

#endif // VIDEO_HPP
