#ifndef VIDEO_HPP
#define VIDEO_HPP

extern "C" { 
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/frame.h>
#include <libavutil/imgutils.h>
#include <libavutil/avutil.h>
#include <libswscale/swscale.h>
}

#include <opencv2/opencv.hpp>
#include <iostream>

#include "parser.hpp"


class VideoProcessor {
    private:
        AVCodecContext* codec_context_;
        const AVCodec* codec_;
        AVFrame* frame_;
        bool initialized_;

        AVBufferRef* hw_device_ctx_;
        SwsContext* sws_ctx_;
        std::vector<cv::Mat> cropped_images_; // Store cropped images
        
        // Post-processing for deblocking
        struct SwsContext* pp_sws_ctx_;
        AVFrame* pp_frame_;

        int width_;
        int height_;
        const double ORIGINAL_WIDTH = 3840.0;   // source resolution width
        const double ORIGINAL_HEIGHT = 2160.0;  // source resolution height

        void cropDetectionBoxes(AVFrame* frame, std::vector<Object>& objects, cv::Point2f user_point = cv::Point2f(1920.0, 1080.0));
        cv::Mat yuv420_to_mat(AVFrame* frame, int x, int y, int width, int height);
        cv::Mat frame_to_mat(AVFrame* frame); // Convert full frame to OpenCV Mat

    public:
        VideoProcessor();
        ~VideoProcessor();
        bool initialize(AVCodecParameters* codecpar);
        void flush();
        std::vector<cv::Mat> processFrameForCropping(AVPacket* pkt, std::vector<Object>& objects);
        cv::Mat processFrameForDisplay(AVPacket* pkt); // Process frame for full display
        std::vector<cv::Mat> getCroppedImages() const { return cropped_images_; };
};


#endif // VIDEO_HPP