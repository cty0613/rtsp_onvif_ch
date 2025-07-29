extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/error.h>
#include <libavutil/time.h>
#include <libavutil/rational.h>
}
#include <tinyxml2.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <unistd.h>
#include <mutex>
#include <string>
#include <deque>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <algorithm>
#include <cmath>
#include <functional>
#include <signal.h>
#include <csignal>
#include <SDL2/SDL.h>
#include <queue>
#include <opencv2/opencv.hpp>

#include "parser.hpp"
#include "video.hpp"
#include "ocr.hpp"
#include "bus_sequence.hpp"
#include <sys/mman.h>
#include <fcntl.h>

#define HANWHA_ORIGINAL_WIDTH 3840.0
#define HANWHA_ORIGINAL_HEIGHT 2160.0

// --- Thread-Safe Queue ---
template<typename T>
class ThreadSafeQueue {
public:
    void push(T value) {
        std::unique_lock<std::mutex> lock(mutex_);
        queue_.push(std::move(value));
        cond_.notify_one();
    }

    bool try_pop(T& value) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return false;
        }
        value = std::move(queue_.front());
        queue_.pop();
        return true;
    }

    T wait_and_pop() {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_.wait(lock, [this] { return !queue_.empty(); });
        T value = std::move(queue_.front());
        queue_.pop();
        return value;
    }

    bool empty() const {
        std::unique_lock<std::mutex> lock(mutex_);
        return queue_.empty();
    }

private:
    std::queue<T> queue_;
    mutable std::mutex mutex_;
    std::condition_variable cond_;
};

// --- Global Variables ---
MetadataParser metadataParser;
VideoProcessor videoProcessor;
TFOCR ocrProcessor;

// Queues for packets from stream
ThreadSafeQueue<AVPacket*> video_packet_queue;
ThreadSafeQueue<AVPacket*> metadata_packet_queue;

// Queues for decoded data
ThreadSafeQueue<AVFrame*> frame_queue;
ThreadSafeQueue<MetadataResult> metadata_queue;

// Queue for cropped frames
ThreadSafeQueue< std::vector<cv::Mat> > cropped_frame_queue;

// Atomics for thread control
std::atomic<bool> run_threads{true};

// Clock sync variables (similar to ffplay)
struct Clock {
    double pts;
    double last_updated;
    double speed;
    int serial;
    bool paused;
};

Clock video_clock = {0.0, 0.0, 1.0, 0, false};
Clock master_clock = {0.0, 0.0, 1.0, 0, false};

// --- Thread Functions ---
void stream_thread(AVFormatContext* formatContext, int data_stream_index, int video_stream_index) {
    AVPacket pkt;
    av_init_packet(&pkt);

    while(run_threads) {
        if (av_read_frame(formatContext, &pkt) >= 0) {
            AVPacket *cloned_pkt = av_packet_clone(&pkt);
            if (pkt.stream_index == video_stream_index) {
                video_packet_queue.push(cloned_pkt);
            } else if (pkt.stream_index == data_stream_index) {
                metadata_packet_queue.push(cloned_pkt);
            } else {
                av_packet_free(&cloned_pkt); // Not a stream we are interested in
            }
            av_packet_unref(&pkt);
        } else {
            run_threads = false; // End of stream or error
            break;
        }
    }
    std::cout << "Stream thread finished." << std::endl;
}

void decode_thread(AVFormatContext* formatContext, int video_stream_index) {
    // Initialize VideoProcessor
    if (!videoProcessor.initialize(formatContext->streams[video_stream_index]->codecpar)) {
        std::cerr << "Failed to initialize VideoProcessor" << std::endl;
        run_threads = false;
        return;
    }

    // Video Codec Setup (for additional processing if needed)
    AVCodecParameters* vid_codecpar = formatContext->streams[video_stream_index]->codecpar;
    AVCodec* codec = avcodec_find_decoder(vid_codecpar->codec_id);
    if (!codec) {
        std::cerr << "Unsupported video codec!" << std::endl;
        run_threads = false;
        return;
    }
    
    AVCodecContext* codec_context = avcodec_alloc_context3(codec);
    if (avcodec_parameters_to_context(codec_context, vid_codecpar) < 0) {
        avcodec_free_context(&codec_context);
        std::cerr << "Could not copy codec parameters to context" << std::endl;
        run_threads = false;
        return;
    }
    
    // Enhanced decoding configuration for I/P frame reference handling
    codec_context->thread_count = 4;
    codec_context->thread_type = FF_THREAD_FRAME | FF_THREAD_SLICE;
    codec_context->error_concealment = FF_EC_GUESS_MVS | FF_EC_DEBLOCK;
    codec_context->err_recognition = AV_EF_CRCCHECK | AV_EF_BITSTREAM | AV_EF_BUFFER;
    codec_context->skip_frame = AVDISCARD_NONE;
    codec_context->flags |= AV_CODEC_FLAG_OUTPUT_CORRUPT;
    
    if (avcodec_open2(codec_context, codec, nullptr) < 0) {
        avcodec_free_context(&codec_context);
        std::cerr << "Could not open codec" << std::endl;
        run_threads = false;
        return;
    }

    AVFrame* frame = av_frame_alloc();
    if (!frame) {
        std::cerr << "Could not allocate frame" << std::endl;
        run_threads = false;
        avcodec_close(codec_context);
        avcodec_free_context(&codec_context);
        return;
    }

    // Frame reference tracking for I/P frame handling
    bool need_keyframe = true;
    int consecutive_errors = 0;
    const int MAX_CONSECUTIVE_ERRORS = 10;

    while (run_threads) {
        // Decode Video Packets
        AVPacket* pkt = nullptr;
        if (video_packet_queue.try_pop(pkt)) {
            // I/P frame reference handling
            if (need_keyframe && !(pkt->flags & AV_PKT_FLAG_KEY)) {
                std::cout << "[DECODE] Waiting for keyframe, dropping P-frame" << std::endl;
                av_packet_free(&pkt);
                continue;
            }
            
            int ret = avcodec_send_packet(codec_context, pkt);
            if (ret < 0) {
                consecutive_errors++;
                if (consecutive_errors > MAX_CONSECUTIVE_ERRORS) {
                    std::cerr << "[DECODE] Too many consecutive errors, flushing decoder" << std::endl;
                    avcodec_flush_buffers(codec_context);
                    need_keyframe = true;
                    consecutive_errors = 0;
                }
                av_packet_free(&pkt);
                continue;
            }
            
            while (avcodec_receive_frame(codec_context, frame) == 0) {
                AVFrame* cloned_frame = av_frame_clone(frame);
                if (cloned_frame) {
                    frame_queue.push(cloned_frame);
                    need_keyframe = false;
                    consecutive_errors = 0;
                }
            }
            av_packet_free(&pkt);
        }

        // Parse Metadata Packets
        AVPacket* meta_pkt = nullptr;
        if (metadata_packet_queue.try_pop(meta_pkt)) {
            metadataParser.processPacket(meta_pkt);
            
            // Get completed metadata results
            std::vector<MetadataResult> results = metadataParser.getCompletedResults();
            for (const auto& result : results) {
                metadata_queue.push(result);
            }
            
            av_packet_free(&meta_pkt);
        }
        
        // Small delay to prevent busy waiting
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    av_frame_free(&frame);
    avcodec_close(codec_context);
    avcodec_free_context(&codec_context);
    std::cout << "Decode thread finished." << std::endl;
}

// Helper functions for clock synchronization
double get_clock(Clock* c) {
    if (c->paused) {
        return c->pts;
    } else {
        double time = av_gettime_relative() / 1000000.0;
        return c->pts + (time - c->last_updated) * c->speed;
    }
}

void set_clock_at(Clock* c, double pts, double time) {
    c->pts = pts;
    c->last_updated = time;
}

void set_clock(Clock* c, double pts) {
    double time = av_gettime_relative() / 1000000.0;
    set_clock_at(c, pts, time);
}

// SDL rendering helper functions
void render_frame_to_sdl(SDL_Renderer* renderer, SDL_Texture* texture, AVFrame* frame) {
    if (!frame || !texture) return;
    
    // Update texture with frame data
    SDL_UpdateYUVTexture(texture, nullptr,
                         frame->data[0], frame->linesize[0],
                         frame->data[1], frame->linesize[1],
                         frame->data[2], frame->linesize[2]);
    
    // Clear renderer and copy texture
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);
    SDL_RenderCopy(renderer, texture, nullptr, nullptr);
}

void render_metadata_overlay(SDL_Renderer* renderer, const MetadataResult& metadata, int width, int height) {
    if (metadata.objects.empty()) return;
    
    std::cout << "[RENDER] Rendering metadata overlay with " << metadata.objects.size() << " objects." << std::endl;
    // Set color for bounding boxes (green)
    SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
    
    for (const auto& obj : metadata.objects) {
        // Convert normalized coordinates to pixel coordinates
        int x1 = static_cast<int>((obj.boundingBox.left / HANWHA_ORIGINAL_WIDTH) * width);
        int y1 = static_cast<int>((obj.boundingBox.top / HANWHA_ORIGINAL_HEIGHT) * height);
        int x2 = static_cast<int>((obj.boundingBox.right / HANWHA_ORIGINAL_WIDTH) * width);
        int y2 = static_cast<int>((obj.boundingBox.bottom / HANWHA_ORIGINAL_HEIGHT) * height);
        
        // Ensure coordinates are within bounds
        x1 = std::max(0, std::min(x1, width - 1));
        y1 = std::max(0, std::min(y1, height - 1));
        x2 = std::max(0, std::min(x2, width - 1));
        y2 = std::max(0, std::min(y2, height - 1));

        // 5px bbox margin
        const int margin = 5;
        x1 = std::max(0, x1 - margin);
        y1 = std::max(0, y1 - margin);
        x2 = std::min(width - 1, x2 + margin);
        y2 = std::min(height - 1, y2 + margin);

        // Draw bounding box
        SDL_Rect rect = {x1, y1, x2 - x1, y2 - y1};
        SDL_RenderDrawRect(renderer, &rect);
        
        // Draw center point
        int cx = static_cast<int>(obj.centerOfGravity.x * width);
        int cy = static_cast<int>(obj.centerOfGravity.y * height);
        SDL_RenderDrawPoint(renderer, cx, cy);
    }
}

std::vector<cv::Mat> render_metadata_crop_to_cv(AVFrame* frame, const MetadataResult& metadata, int width, int height, cv::Point2f reference_point = cv::Point2f(-1, -1)) {
    std::vector<cv::Mat> cropped_frames;
    
    if (metadata.objects.empty() || !frame) {
        return cropped_frames;
    }
    
    // If reference point is not provided, use center of screen as default
    if (reference_point.x < 0 || reference_point.y < 0) {
        reference_point = cv::Point2f(width / 2.0f, height / 2.0f);
    }
    
    // Convert AVFrame (YUV420P) to OpenCV Mat (BGR)
    cv::Mat yuv_mat(height + height/2, width, CV_8UC1, frame->data[0]);
    cv::Mat bgr_mat;
    cv::cvtColor(yuv_mat, bgr_mat, cv::COLOR_YUV2BGR_I420);
    
    // Structure to hold cropped frame and its distance
    struct CroppedFrameWithDistance {
        cv::Mat frame;
        float distance;
    };
    
    std::vector<CroppedFrameWithDistance> frames_with_distance;
    
    for (const auto& obj : metadata.objects) {
        // Convert normalized coordinates to pixel coordinates
        int x1 = static_cast<int>((obj.boundingBox.left / HANWHA_ORIGINAL_WIDTH) * width);
        int y1 = static_cast<int>((obj.boundingBox.top / HANWHA_ORIGINAL_HEIGHT) * height);
        int x2 = static_cast<int>((obj.boundingBox.right / HANWHA_ORIGINAL_WIDTH) * width);
        int y2 = static_cast<int>((obj.boundingBox.bottom / HANWHA_ORIGINAL_HEIGHT) * height);

        // Ensure coordinates are within bounds
        x1 = std::max(0, std::min(x1, width - 1));
        y1 = std::max(0, std::min(y1, height - 1));
        x2 = std::max(0, std::min(x2, width - 1));
        y2 = std::max(0, std::min(y2, height - 1));

        // 5px bbox margin
        const int margin = 8;
        x1 = std::max(0, x1 - margin);
        y1 = std::max(0, y1 - margin);
        x2 = std::min(width - 1, x2 + margin);
        y2 = std::min(height - 1, y2 + margin);

        // Ensure width and height are positive
        int crop_width = x2 - x1;
        int crop_height = y2 - y1;
        
        if (crop_width > 0 && crop_height > 0) {
            // Create ROI (Region of Interest) and crop
            cv::Rect crop_rect(x1, y1, crop_width, crop_height);
            cv::Mat cropped_frame = bgr_mat(crop_rect).clone();
            
            // Calculate center of gravity in pixel coordinates
            cv::Point2f center_of_gravity(obj.centerOfGravity.x * width, obj.centerOfGravity.y * height);
            
            // Calculate distance from reference point to center of gravity
            float distance = cv::norm(reference_point - center_of_gravity);
            
            // Store frame with its distance
            frames_with_distance.push_back({cropped_frame, distance});
        } else {
            std::cout << "[RENDER] Invalid crop dimensions: " << crop_width << "x" << crop_height << std::endl;
        }
    }
    
    // Sort by distance (closest first)
    std::sort(frames_with_distance.begin(), frames_with_distance.end(),
              [](const CroppedFrameWithDistance& a, const CroppedFrameWithDistance& b) {
                  return a.distance < b.distance;
              });
    
    // Extract sorted frames
    for (const auto& frame_with_dist : frames_with_distance) {
        cropped_frames.push_back(frame_with_dist.frame);
    }
    
    return cropped_frames;
}

void render_thread(AVFormatContext* formatContext, int video_stream_index) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL could not initialize! SDL_Error: " << SDL_GetError() << std::endl;
        return;
    }

    AVCodecParameters* vid_codecpar = formatContext->streams[video_stream_index]->codecpar;
    int width = vid_codecpar->width;
    int height = vid_codecpar->height;
    
    SDL_Window* window = SDL_CreateWindow("RTSP Stream with Metadata", 
                                         SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 
                                         width, height, SDL_WINDOW_SHOWN);
    if (!window) {
        std::cerr << "Window could not be created! SDL_Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return;
    }
    
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        std::cerr << "Renderer could not be created! SDL_Error: " << SDL_GetError() << std::endl;
        SDL_DestroyWindow(window);
        SDL_Quit();
        return;
    }
    
    SDL_Texture* texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_IYUV, 
                                           SDL_TEXTUREACCESS_STREAMING, width, height);
    if (!texture) {
        std::cerr << "Texture could not be created! SDL_Error: " << SDL_GetError() << std::endl;
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return;
    }

    AVRational time_base = formatContext->streams[video_stream_index]->time_base;
    
    // Frame timing variables (similar to ffplay)
    double frame_timer = av_gettime_relative() / 1000000.0;
    double frame_last_delay = 0.02; // 25fps default
    const double AV_SYNC_THRESHOLD_MIN = 0.04;
    const double AV_SYNC_THRESHOLD_MAX = 0.1;
    const double AV_SYNC_FRAMEDUP_THRESHOLD = 0.1;
    
    // Metadata sync buffer
    std::vector<MetadataResult> metadata_buffer;
    cv::Point2f custom_point(width * 0.5f, height * 1.0f); // N% from left, M% from top
    
    while (run_threads) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                run_threads = false;
                break;
            }
        }
        
        AVFrame* frame = nullptr;
        if (frame_queue.try_pop(frame)) {
            // Calculate frame PTS in seconds
            double frame_pts = frame->pts * av_q2d(time_base);
            
            // Update video clock
            set_clock(&video_clock, frame_pts);
            
            // Collect recent metadata for synchronization
            MetadataResult current_metadata;
            bool metadata_found = false;
            
            // Try to get metadata close to current frame PTS
            while (metadata_queue.try_pop(current_metadata)) {
                metadata_buffer.push_back(current_metadata);
            }
            
            // Find best matching metadata based on PTS
            if (!metadata_buffer.empty()) {
                auto best_match = std::min_element(metadata_buffer.begin(), metadata_buffer.end(),
                    [frame_pts, &time_base](const MetadataResult& a, const MetadataResult& b) {
                        double a_pts = a.pts * av_q2d(time_base);
                        double b_pts = b.pts * av_q2d(time_base);
                        return std::abs(a_pts - frame_pts) < std::abs(b_pts - frame_pts);
                    });
                
                double metadata_pts = best_match->pts * av_q2d(time_base);
                if (std::abs(metadata_pts - frame_pts) < 0.05) { // 100ms tolerance
                    current_metadata = *best_match;
                    metadata_found = true;
                }
                
                // Clean old metadata (keep only recent ones)
                metadata_buffer.erase(
                    std::remove_if(metadata_buffer.begin(), metadata_buffer.end(),
                        [frame_pts, &time_base](const MetadataResult& meta) {
                            double meta_pts = meta.pts * av_q2d(time_base);
                            return (frame_pts - meta_pts) > 1.0; // Remove metadata older than 1 second
                        }),
                    metadata_buffer.end());
            }
            
            // Calculate timing for frame display (ffplay-style)
            double current_time = av_gettime_relative() / 1000000.0;
            double delay = frame_pts - get_clock(&master_clock);
            
            // Adjust delay based on sync threshold
            if (delay <= -AV_SYNC_THRESHOLD_MAX) {
                // Frame is too late, drop it
                std::cout << "[RENDER] Dropping late frame, delay: " << delay << std::endl;
                av_frame_free(&frame);
                continue;
            } else if (delay >= AV_SYNC_FRAMEDUP_THRESHOLD) {
                // Frame is too early, duplicate previous frame
                delay = frame_last_delay;
            } else if (delay >= AV_SYNC_THRESHOLD_MIN) {
                // Normal case, use calculated delay
                frame_last_delay = delay;
            } else {
                // Frame is slightly early, use minimum delay
                delay = AV_SYNC_THRESHOLD_MIN;
                frame_last_delay = delay;
            }
            
            // Wait for the right time to display the frame
            if (delay > 0) {
                SDL_Delay(static_cast<Uint32>(delay * 1000));
            }
            
            // Update master clock
            set_clock(&master_clock, frame_pts);
            
            // Render frame and metadata
            render_frame_to_sdl(renderer, texture, frame);
            if (metadata_found) {
                // render_metadata_overlay(renderer, current_metadata, width, height);

                // Get cropped frames from metadata bounding boxes
                std::vector<cv::Mat> cropped_frames = render_metadata_crop_to_cv(frame, current_metadata, width, height, custom_point);
                cropped_frame_queue.push(cropped_frames); // Vector is already copied by value
            }
            
            SDL_RenderPresent(renderer);
            av_frame_free(&frame);
            
        } else {
            // No frame available, wait a bit
            SDL_Delay(10);
        }
    }

    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    std::cout << "Render thread finished." << std::endl;
}

void ocr_thread() {
    std::cout << "[OCR] OCR thread started." << std::endl;
    ocrProcessor.load_ocr("model.tflite", "labels.names");
    
    // Initialize shared memory
    const char * shm_name = "/bus_approach";
    const size_t shm_size = 4096;
    void* shm_ptr;

    if (shm_name != nullptr && strlen(shm_name) > 0 && shm_size > 0) {
        int shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
        if (shm_fd == -1) {
            std::cerr << "Failed to open shared memory: " << strerror(errno) << std::endl;
        } else {
            if (ftruncate(shm_fd, shm_size) == -1) {
                std::cerr << "Failed to set shared memory size: " << strerror(errno) << std::endl;
                close(shm_fd);
            } else {
                shm_ptr = mmap(nullptr, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
                if (shm_ptr == MAP_FAILED) {
                    std::cerr << "Failed to map shared memory: " << strerror(errno) << std::endl;
                    shm_ptr = nullptr;
                }
                close(shm_fd);
                std::cout << "Shared memory initialized: " << shm_name << " (" << shm_size << " bytes)" << std::endl;
            }
        }
    }

    int frame_counter = 0;
    
    while(run_threads) {
        std::vector<cv::Mat> cropped_frames;
        std::vector<std::string> ocr_results;
        if (cropped_frame_queue.try_pop(cropped_frames)) {
            if (!cropped_frames.empty()) {
                for (const auto& cropped_frame : cropped_frames) {
                    TFOCR::OCRResult result = ocrProcessor.run_ocr(cropped_frame);
                    if (result.label.empty()) {
                        // std::cout << "[OCR] Low confidence or empty label, skipping" << std::endl;
                        continue;
                    }
                    // Check if the result already exists in ocr_results (no duplicates allowed)
                    if (std::find(ocr_results.begin(), ocr_results.end(), result.label) == ocr_results.end()) {
                        std::cout << "[OCR] Detected license plate: " << result.label << " conf : " << result.confidence << std::endl;
                        ocr_results.push_back(result.label);
                    } else {
                        std::cout << "[OCR] Duplicate license plate skipped: " << result.label << " conf : " << result.confidence << std::endl;
                    }
                    
                }
                frame_counter++;
            }
        } else {
            // No frame available, wait a bit
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // If shared memory is initialized, write results
        if (shm_ptr != nullptr) {
            BusSequence* bus_sequence = static_cast<BusSequence*>(shm_ptr);
            
            // Read existing plates from shared memory
            std::vector<std::string> existing_plates;
            for (int i = 0; i < MAX_BUSES; ++i) {
                if (strlen(bus_sequence->plates[i]) > 0) {
                    existing_plates.push_back(std::string(bus_sequence->plates[i]));
                }
            }
            
            // Filter out duplicates from ocr_results that already exist in shared memory
            std::vector<std::string> new_plates;
            for (const auto& result : ocr_results) {
                // Check if this result already exists in shared memory
                if (std::find(existing_plates.begin(), existing_plates.end(), result) == existing_plates.end()) {
                    new_plates.push_back(result);
                } else {
                    std::cout << "[OCR] Plate already in SHM, skipping: " << result << std::endl;
                }
            }
            
            // Create final list: keep existing plates and add new unique ones
            std::vector<std::string> final_plates = existing_plates;
            for (const auto& new_plate : new_plates) {
                if (final_plates.size() < MAX_BUSES) {
                    final_plates.push_back(new_plate);
                    std::cout << "[OCR] Added new plate to SHM: " << new_plate << std::endl;
                } else {
                    std::cout << "[OCR] SHM full, cannot add: " << new_plate << std::endl;
                    break;
                }
            }
            
            // Clear and write final results to shared memory
            std::memset(bus_sequence, 0, sizeof(BusSequence));
            size_t count = std::min(final_plates.size(), static_cast<size_t>(MAX_BUSES));
            for(size_t i = 0; i < count; ++i) {
                strncpy(bus_sequence->plates[i], final_plates[i].c_str(), MAX_PLATE_LENGTH - 1);
            }
        }
    }
}

int main() {

    const char* url = "rtsp://192.168.0.64/profile2/media.smp";

    avformat_network_init();
    AVFormatContext *formatContext = nullptr;

    AVDictionary *opts = nullptr;
    av_dict_set(&opts, "rtsp_transport", "tcp", 0);
    av_dict_set(&opts, "max_delay",      "500000", 0);
    av_dict_set(&opts, "buffer_size", "1024000", 0);
    av_dict_set(&opts, "stimeout", "5000000", 0);

    if (avformat_open_input(&formatContext, url, nullptr, &opts) < 0) {
        std::cerr << "Failed to open input" << std::endl;
        return -1;
    }
    av_dict_free(&opts);

    if (avformat_find_stream_info(formatContext, nullptr) < 0) {
        std::cerr << "Failed to find stream info" << std::endl;
        avformat_close_input(&formatContext);
        return -1;
    }

    int data_stream_index = -1, video_stream_index = -1;
    for (unsigned int i = 0; i < formatContext->nb_streams; i++) {
        if (formatContext->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_DATA) {
            data_stream_index = i;
            std::cout << "Found data stream at index " << i << std::endl;
        } else if (formatContext->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            video_stream_index = i;
            std::cout << "Found video stream at index " << i << std::endl;
        }
    }

    if (data_stream_index == -1 || video_stream_index == -1) {
        std::cerr << "Could not find required video and data streams" << std::endl;
        avformat_close_input(&formatContext);
        return -1;
    }

    // Initialize clocks
    double start_time = av_gettime_relative() / 1000000.0;
    set_clock_at(&video_clock, 0.0, start_time);
    set_clock_at(&master_clock, 0.0, start_time);

    std::cout << "Starting threads..." << std::endl;
    std::thread streamThread(stream_thread, formatContext, data_stream_index, video_stream_index);
    std::thread decodeThread(decode_thread, formatContext, video_stream_index);
    std::thread renderThread(render_thread, formatContext, video_stream_index);
    std::thread ocrThread(ocr_thread);

    // std::cout << "Press Enter to stop..." << std::endl;
    getchar();
    
    std::cout << "Stopping threads..." << std::endl;
    run_threads = false;

    streamThread.join();
    decodeThread.join();
    renderThread.join();
    ocrThread.join();

    avformat_close_input(&formatContext);
    avformat_network_deinit();

    std::cout << "Application finished." << std::endl;

    return 0;
}
