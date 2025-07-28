#include "video.hpp"

// Constructor
VideoProcessor::VideoProcessor() : codec_context_(nullptr), codec_(nullptr), 
    frame_(nullptr), initialized_(false), width_(0), height_(0), hw_device_ctx_(nullptr), 
    sws_ctx_(nullptr), pp_sws_ctx_(nullptr), pp_frame_(nullptr) {
}

// Destructor
VideoProcessor::~VideoProcessor() {
    if (frame_) {
        av_frame_free(&frame_);
    }
    if (pp_frame_) {
        av_frame_free(&pp_frame_);
    }
    if (codec_context_) {
        avcodec_free_context(&codec_context_);
    }
    if (hw_device_ctx_) {
        av_buffer_unref(&hw_device_ctx_);
    }
    if (sws_ctx_) {
        sws_freeContext(sws_ctx_);
    }
    if (pp_sws_ctx_) {
        sws_freeContext(pp_sws_ctx_);
    }
}

// intialize video processor with codec parameters
bool VideoProcessor::initialize(AVCodecParameters* codecpar) {
    if (initialized_) {
        return true;
    }

    codec_ = avcodec_find_decoder(codecpar->codec_id);
    if (!codec_) {
        std::cerr << "Unsupported codec!" << std::endl;
        return false;
    }

    codec_context_ = avcodec_alloc_context3(codec_);
    if (!codec_context_) {
        std::cerr << "Could not allocate video codec context" << std::endl;
        return false;
    }
    
    // Enhanced decoding configuration for better quality (similar to ffplay)
    codec_context_->thread_count = 0; // Auto-detect optimal thread count
    codec_context_->thread_type = FF_THREAD_FRAME | FF_THREAD_SLICE; // Both types
    codec_context_->error_concealment = FF_EC_GUESS_MVS | FF_EC_DEBLOCK; // Error concealment
    codec_context_->err_recognition = AV_EF_CRCCHECK | AV_EF_BITSTREAM | AV_EF_BUFFER; // Enhanced error recognition
    codec_context_->skip_frame = AVDISCARD_NONE; // Don't skip any frames
    codec_context_->skip_idct = AVDISCARD_NONE; // Don't skip IDCT
    codec_context_->skip_loop_filter = AVDISCARD_NONE; // Don't skip loop filter (important for block artifacts)
    codec_context_->flags |= AV_CODEC_FLAG_OUTPUT_CORRUPT; // Output corrupt frames for analysis
    codec_context_->flags2 |= AV_CODEC_FLAG2_CHUNKS; // Handle incomplete frames better
    codec_context_->flags2 |= AV_CODEC_FLAG2_SHOW_ALL; // Show all frames
    
    // Additional H.264 specific optimizations
    codec_context_->workaround_bugs = FF_BUG_AUTODETECT; // Auto-detect and workaround bugs
    codec_context_->strict_std_compliance = FF_COMPLIANCE_NORMAL;

    // AVHWDeviceType hw_type = av_hwdevice_find_type_by_name("v4l2_m2m");
    // if(av_hwdevice_ctx_create(
    //     &hw_device_ctx_, hw_type, nullptr, nullptr, 0
    // ) >= 0) {
    //     codec_context_->hw_device_ctx = av_buffer_ref(hw_device_ctx_);
    //     if (!codec_context_->hw_device_ctx) {
    //         std::cerr << "Could not create hardware device context" << std::endl;
    //         return false;
    //     }
    // } else {
    //     std::cerr << "Could not create hardware device context" << std::endl;
    //     return false;
    // }

    if (avcodec_parameters_to_context(codec_context_, codecpar) < 0) {
        std::cerr << "Could not copy codec parameters to context" << std::endl;
        return false;
    }

    if (avcodec_open2(codec_context_, codec_, nullptr) < 0) {
        std::cerr << "Could not open codec" << std::endl;
        return false;
    }

    width_ = codec_context_->width;
    height_ = codec_context_->height;

    frame_ = av_frame_alloc();
    if (!frame_) {
        std::cerr << "Could not allocate frame" << std::endl;
        return false;
    }

    // Check pixel format and set default if needed
    if (codec_context_->pix_fmt == AV_PIX_FMT_NONE) {
        std::cout << "Pixel format not set, using YUV420P as default" << std::endl;
        codec_context_->pix_fmt = AV_PIX_FMT_YUV420P;
    }
    
    // Handle YUVJ420P (full range) properly
    AVPixelFormat target_pix_fmt = codec_context_->pix_fmt;
    if (codec_context_->pix_fmt == AV_PIX_FMT_YUVJ420P) {
        std::cout << "Converting YUVJ420P to YUV420P with full range" << std::endl;
        target_pix_fmt = AV_PIX_FMT_YUV420P;
    }

    sws_ctx_ = sws_getContext(
        width_, height_, target_pix_fmt,  // 입력 포맷
        width_, height_, AV_PIX_FMT_BGR24,         // 출력 포맷
        SWS_LANCZOS | SWS_FULL_CHR_H_INT | SWS_FULL_CHR_H_INP | SWS_ACCURATE_RND, // 고품질 보간
        nullptr, nullptr, nullptr
    );

    if (!sws_ctx_) {
        std::cerr << "Could not initialize SwsContext" << std::endl;
        std::cerr << "Input format: " << av_get_pix_fmt_name(codec_context_->pix_fmt) << std::endl;
        std::cerr << "Target format: " << av_get_pix_fmt_name(target_pix_fmt) << std::endl;
        std::cerr << "Dimensions: " << width_ << "x" << height_ << std::endl;
        return false;
    }
    
    // Set color range and space for proper conversion
    if (codec_context_->pix_fmt == AV_PIX_FMT_YUVJ420P) {
        const int *inv_table, *table;
        int srcRange, dstRange, brightness, contrast, saturation;
        
        inv_table = sws_getCoefficients(SWS_CS_ITU709);
        sws_setColorspaceDetails(sws_ctx_, inv_table, 1, // src: full range
                               inv_table, 0,             // dst: limited range  
                               0, 1 << 16, 1 << 16);     // brightness, contrast, saturation
    }

    initialized_ = true;
    std::cout << "Video processor initialized successfully" << std::endl;
    std::cout << "Video resolution: " << width_ << "x" << height_ << std::endl;
    return true;
}

void VideoProcessor::flush() {
    // 디코더에게 NULL 패킷 보내서 내부 버퍼 플러시
    avcodec_send_packet(codec_context_, nullptr);
    while (true) {
        int ret = avcodec_receive_frame(codec_context_, frame_);
        if (ret == AVERROR_EOF || ret == AVERROR(EAGAIN)) break; // 플러시 완료
    }
}

// Process frame for cropping only (no SDL rendering)
std::vector<cv::Mat> VideoProcessor::processFrameForCropping(AVPacket* pkt, std::vector<Object>& objects) {
    if (!initialized_) {
        return {};
    }

    if (!pkt) {
        std::cerr << "Null packet received" << std::endl;
        return {};
    }

    int ret = avcodec_send_packet(codec_context_, pkt);
    if (ret < 0) {
        return {};
    }

    while (ret >= 0) {
        ret = avcodec_receive_frame(codec_context_, frame_);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
            break;
        } else if (ret < 0) {
            std::cerr << "Error during decoding" << std::endl;
            break;
        }

        std::cout << "[VIDEO] Frame received with PTS: " << frame_->pts << std::endl;
        cropDetectionBoxes(frame_, objects);
    }

    return cropped_images_;
}

// Process frame for full display (like ffplay)
cv::Mat VideoProcessor::processFrameForDisplay(AVPacket* pkt) {
    if (!initialized_) {
        return cv::Mat();
    }

    if (!pkt) {
        std::cerr << "Null packet received" << std::endl;
        return cv::Mat();
    }

    int ret = avcodec_send_packet(codec_context_, pkt);
    if (ret < 0) {
        char errbuf[AV_ERROR_MAX_STRING_SIZE];
        av_strerror(ret, errbuf, AV_ERROR_MAX_STRING_SIZE);
        std::cerr << "[VIDEO] Error sending packet: " << errbuf << std::endl;
        
        // Try to recover from error
        if (ret == AVERROR_INVALIDDATA) {
            std::cout << "[VIDEO] Invalid data, trying to recover..." << std::endl;
            avcodec_flush_buffers(codec_context_);
            return cv::Mat();
        }
        return cv::Mat();
    }

    while (ret >= 0) {
        ret = avcodec_receive_frame(codec_context_, frame_);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
            break;
        } else if (ret < 0) {
            char errbuf[AV_ERROR_MAX_STRING_SIZE];
            av_strerror(ret, errbuf, AV_ERROR_MAX_STRING_SIZE);
            std::cerr << "[VIDEO] Error during decoding: " << errbuf << std::endl;
            
            // Don't break on decode error, try to continue
            if (ret == AVERROR_INVALIDDATA) {
                std::cout << "[VIDEO] Corrupt frame data, skipping..." << std::endl;
                continue;
            }
            break;
        }

        std::cout << "[VIDEO] Frame received with PTS: " << frame_->pts << std::endl;
        
        // Validate frame data before processing
        if (!frame_->data[0] || frame_->width <= 0 || frame_->height <= 0) {
            std::cerr << "[VIDEO] Invalid frame data" << std::endl;
            continue;
        }
        
        return frame_to_mat(frame_);
    }

    return cv::Mat();
}

// Convert full frame to OpenCV Mat with high quality
cv::Mat VideoProcessor::frame_to_mat(AVFrame* frame) {
    if (!frame || !sws_ctx_) {
        return cv::Mat();
    }
    
    // Create OpenCV Mat for BGR output
    cv::Mat bgr_frame(height_, width_, CV_8UC3);
    
    // Convert using SwsContext with proper alignment
    uint8_t* dst_data[4] = { bgr_frame.data, nullptr, nullptr, nullptr };
    int dst_linesize[4] = { static_cast<int>(bgr_frame.step[0]), 0, 0, 0 };
    
    // Ensure proper frame alignment
    if (frame->linesize[0] < width_ * 1) {
        std::cerr << "[VIDEO] Frame linesize too small: " << frame->linesize[0] << std::endl;
        return cv::Mat();
    }
    
    int result = sws_scale(sws_ctx_, 
                          (const uint8_t* const*)frame->data, frame->linesize, 
                          0, height_, dst_data, dst_linesize);
    
    if (result != height_) {
        std::cerr << "[VIDEO] sws_scale failed, expected " << height_ << " lines, got " << result << std::endl;
        return cv::Mat();
    }
    
    // Apply deblocking filter using OpenCV to reduce macroblock artifacts
    cv::Mat deblocked_frame;
    // Use bilateral filter to reduce block artifacts while preserving edges
    cv::bilateralFilter(bgr_frame, deblocked_frame, 9, 75, 75);
    
    return deblocked_frame;
}



// Convert YUV420P region to OpenCV Mat (BGR format) with improved quality
cv::Mat VideoProcessor::yuv420_to_mat(AVFrame* frame, int x, int y, int width, int height) {
    // Ensure coordinates are within frame bounds and aligned for YUV420
    x = std::max(0, std::min(x & ~1, frame->width - width));  // Align to even pixel
    y = std::max(0, std::min(y & ~1, frame->height - height)); // Align to even pixel  
    width = std::min(width & ~1, frame->width - x);   // Ensure even width
    height = std::min(height & ~1, frame->height - y); // Ensure even height
    
    if (width <= 0 || height <= 0) {
        return cv::Mat();
    }
    
    // Use SwsContext for high-quality conversion instead of manual copying
    SwsContext* crop_sws_ctx = sws_getContext(
        width, height, codec_context_->pix_fmt,
        width, height, AV_PIX_FMT_BGR24,
        SWS_LANCZOS | SWS_FULL_CHR_H_INT | SWS_FULL_CHR_H_INP | SWS_ACCURATE_RND,
        nullptr, nullptr, nullptr
    );
    
    if (!crop_sws_ctx) {
        std::cerr << "Could not create SwsContext for cropping" << std::endl;
        return cv::Mat();
    }
    
    // Create temporary frame for cropped region
    AVFrame* crop_frame = av_frame_alloc();
    if (!crop_frame) {
        sws_freeContext(crop_sws_ctx);
        return cv::Mat();
    }
    
    crop_frame->format = codec_context_->pix_fmt;
    crop_frame->width = width;
    crop_frame->height = height;
    
    if (av_frame_get_buffer(crop_frame, 32) < 0) {
        av_frame_free(&crop_frame);
        sws_freeContext(crop_sws_ctx);
        return cv::Mat();
    }
    
    // Copy cropped region from original frame
    if (codec_context_->pix_fmt == AV_PIX_FMT_YUV420P || 
        codec_context_->pix_fmt == AV_PIX_FMT_YUVJ420P) {
        
        // Copy Y plane
        for (int i = 0; i < height; i++) {
            memcpy(crop_frame->data[0] + i * crop_frame->linesize[0], 
                   frame->data[0] + (y + i) * frame->linesize[0] + x, 
                   width);
        }
        
        // Copy U and V planes (subsampled)
        int uv_x = x / 2;
        int uv_y = y / 2;
        int uv_width = width / 2;
        int uv_height = height / 2;
        
        for (int i = 0; i < uv_height; i++) {
            memcpy(crop_frame->data[1] + i * crop_frame->linesize[1],
                   frame->data[1] + (uv_y + i) * frame->linesize[1] + uv_x,
                   uv_width);
            memcpy(crop_frame->data[2] + i * crop_frame->linesize[2],
                   frame->data[2] + (uv_y + i) * frame->linesize[2] + uv_x,
                   uv_width);
        }
    }
    
    // Create OpenCV Mat for BGR output
    cv::Mat bgr_crop(height, width, CV_8UC3);
    
    // Convert using SwsContext
    uint8_t* dst_data[4] = { bgr_crop.data, nullptr, nullptr, nullptr };
    int dst_linesize[4] = { static_cast<int>(bgr_crop.step[0]), 0, 0, 0 };
    
    sws_scale(crop_sws_ctx, crop_frame->data, crop_frame->linesize, 
              0, height, dst_data, dst_linesize);
    
    // Cleanup
    av_frame_free(&crop_frame);
    sws_freeContext(crop_sws_ctx);
    
    // Apply deblocking for cropped image as well
    cv::Mat deblocked_crop;
    if (bgr_crop.rows > 50 && bgr_crop.cols > 50) { // Only for larger crops
        cv::bilateralFilter(bgr_crop, deblocked_crop, 5, 50, 50);
        return deblocked_crop;
    }
    
    return bgr_crop;
}

void VideoProcessor::cropDetectionBoxes(AVFrame* frame, std::vector<Object>& objects, cv::Point2f user_point) {
    
    std::vector<cv::Mat> cropped_images;
    std::cout << "[VIDEO] Cropping " << objects.size() << " detection boxes" << std::endl;
    
    for (const auto& obj : objects) {
        // Scale bounding box coordinates to current resolution
        int x1 = static_cast<int>((obj.boundingBox.left / ORIGINAL_WIDTH) * width_);
        int y1 = static_cast<int>((obj.boundingBox.top / ORIGINAL_HEIGHT) * height_);
        int x2 = static_cast<int>((obj.boundingBox.right / ORIGINAL_WIDTH) * width_);
        int y2 = static_cast<int>((obj.boundingBox.bottom / ORIGINAL_HEIGHT) * height_);
        
        // Ensure coordinates are within image bounds
        x1 = std::max(0, std::min(x1, width_ - 1));
        y1 = std::max(0, std::min(y1, height_ - 1));
        x2 = std::max(0, std::min(x2, width_ - 1));
        y2 = std::max(0, std::min(y2, height_ - 1));
        
        // Add 5px margin around the bounding box
        const int margin = 5;
        x1 = std::max(0, x1 - margin);
        y1 = std::max(0, y1 - margin);
        x2 = std::min(width_ - 1, x2 + margin);
        y2 = std::min(height_ - 1, y2 + margin);
        
        // Ensure valid rectangle dimensions
        if (x2 <= x1 || y2 <= y1) {
            continue; // Skip invalid rectangles
        }
        
        // Convert YUV region to BGR Mat directly
        int crop_width = x2 - x1;
        int crop_height = y2 - y1;
        cv::Mat cropped_mat = yuv420_to_mat(frame, x1, y1, crop_width, crop_height);
        
        if (!cropped_mat.empty()) {
            cropped_images.push_back(cropped_mat);
            std::cout << "[VIDEO] Cropped image " << cropped_images.size() << ": " << crop_width << "x" << crop_height << std::endl;
        }
    }

    cropped_images_ = cropped_images; // Store the cropped images
    std::cout << "[VIDEO] Total cropped images: " << cropped_images_.size() << std::endl;
    
}



