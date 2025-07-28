extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/error.h>
}
#include <tinyxml2.h>
#include <iostream>
#include <vector>
#include <chrono>
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

#include "parser.hpp"
#include "video.hpp"

MetadataParser parser;
VideoProcessor videoProcessor;


// Buffers for packets and metadata results
std::deque<AVPacket*> video_buf_;
std::deque<MetadataResult> metadata_results_;

// Synchronization primitives
std::mutex buf_mutex_;
std::condition_variable buf_cond_;
std::atomic<bool> stopSync{false};
std::atomic<bool> interrupted{false};

// Signal handler for Ctrl+C
void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\n[SIGNAL] Received SIGINT (Ctrl+C), shutting down gracefully..." << std::endl;
        interrupted = true;
        stopSync = true;
        buf_cond_.notify_all();
    }
}

// Sync thread function
void sync_thread() {
    std::cout << "[SYNC] Starting sync thread" << std::endl;

    while (!stopSync) {
        std::unique_lock<std::mutex> lock(buf_mutex_);

        buf_cond_.wait_for(lock, std::chrono::milliseconds(100), [] {
            return stopSync || interrupted || (!video_buf_.empty() && !metadata_results_.empty());
        });
        
        if (stopSync || interrupted) break;

        while (!video_buf_.empty() && !metadata_results_.empty()) {
            int64_t video_pts = video_buf_.front()->pts;
            int64_t metadata_pts = metadata_results_.front().pts;
            int64_t diff = std::abs(video_pts - metadata_pts);

            if (diff < 3000) {
                // Process synchronized packet and metadata
                AVPacket* video_pkt = video_buf_.front();
                video_buf_.pop_front();
                
                std::cout << "[SYNC] Processing video packet vPTS: " << video_pts << " dPTS: " << metadata_pts << std::endl;

                std::vector<Object> objects;
                if (video_pts < metadata_pts) {
                    // Video is faster - use current metadata
                    objects = metadata_results_.front().objects;
                    // Remove processed metadata result
                    metadata_results_.pop_front();
                } else {
                    // Metadata is faster or equal - find matching metadata
                    auto it = std::find_if(metadata_results_.begin(), metadata_results_.end(),
                        [video_pts](const MetadataResult& result) {
                            return result.pts == video_pts;
                        });
                    
                    if (it != metadata_results_.end()) {
                        objects = it->objects;
                        // Remove the matched metadata result
                        metadata_results_.erase(it);
                    } else {
                        // No exact match, use latest metadata
                        objects = metadata_results_.front().objects;
                        // Remove processed metadata result
                        metadata_results_.pop_front();
                    }
                }

                // Unlock mutex before processing video (time-consuming operation)
                lock.unlock();

                // Process the video packet with objects (outside of lock)
                std::vector<cv::Mat> cropped_images = videoProcessor.processFrameForCropping(video_pkt, objects);
                
                // Also display full frame like ffplay (only if GUI available)
                cv::Mat full_frame = videoProcessor.processFrameForDisplay(video_pkt);
                if (!full_frame.empty()) {
                    // Draw bounding boxes on full frame
                    for (const auto& obj : objects) {
                        int x1 = static_cast<int>((obj.boundingBox.left / 3840.0) * full_frame.cols);
                        int y1 = static_cast<int>((obj.boundingBox.top / 2160.0) * full_frame.rows);
                        int x2 = static_cast<int>((obj.boundingBox.right / 3840.0) * full_frame.cols);
                        int y2 = static_cast<int>((obj.boundingBox.bottom / 2160.0) * full_frame.rows);
                        
                        cv::rectangle(full_frame, cv::Point(x1, y1), cv::Point(x2, y2), 
                                    cv::Scalar(0, 255, 0), 2);
                        
                        // Add object type text
                        cv::putText(full_frame, obj.typeName, cv::Point(x1, y1 - 10), 
                                  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                    }
                    
                    // Try to show full frame (skip if no GUI available)
                    try {
                        cv::Mat display_frame;
                        if (full_frame.cols > 1280 || full_frame.rows > 720) {
                            cv::resize(full_frame, display_frame, cv::Size(1280, 720));
                        } else {
                            display_frame = full_frame;
                        }
                        
                        cv::imshow("RTSP Stream - Full Frame", display_frame);
                        cv::waitKey(1); // Non-blocking wait
                        
                    } catch (const cv::Exception& e) {
                        std::cout << "[VIDEO] GUI not available, saving frame to file instead" << std::endl;
                        static int frame_count = 0;
                        std::string filename = "frame_" + std::to_string(frame_count) + ".jpg";
                        cv::imwrite(filename, full_frame);
                        frame_count++;
                    }
                }
                
                // Log cropped images info
                std::cout << "[SYNC] Generated " << cropped_images.size() << " cropped images" << std::endl;
                for (size_t i = 0; i < cropped_images.size(); ++i) {
                    cv::imwrite("cropped_image_" + std::to_string(i) + ".jpg", cropped_images[i]);
                }
                // Clean up video packet
                av_packet_unref(video_pkt);
                av_packet_free(&video_pkt);

                // Re-lock for next iteration
                lock.lock();

            } else {
                // If diff is too large, discard the older packet
                if (video_pts < metadata_pts) {
                    std::cout << "[SYNC] PTS DIFF TOO LARGE!! Discarding video packet PTS: " << video_pts << std::endl;
                    AVPacket* video_pkt = video_buf_.front();
                    video_buf_.pop_front();
                    av_packet_unref(video_pkt);
                    av_packet_free(&video_pkt);
                } else {
                    std::cout << "[SYNC] PTS DIFF TOO LARGE!! Discarding metadata result PTS: " << metadata_pts << std::endl;
                    metadata_results_.pop_front();
                }
            }
        }
    }

    // Cleanup remaining packets when stopping (lock is still held)
    while (!video_buf_.empty()) {
        AVPacket* video_pkt = video_buf_.front();
        video_buf_.pop_front();
        av_packet_unref(video_pkt);
        av_packet_free(&video_pkt);
    }
    metadata_results_.clear();
    
    std::cout << "[SYNC] Sync thread terminated" << std::endl;
}

int main() {
    // Install signal handler for graceful shutdown
    signal(SIGINT, signal_handler);
    
    avformat_network_init();

    AVFormatContext *formatContext = nullptr;
    AVPacket pkt;

    int ret;
    int packet_count = 0;
    
    const char* url = "rtsp://192.168.0.64/profile2/media.smp";

    AVDictionary *opts = nullptr;
    av_dict_set(&opts, "rtsp_transport", "tcp", 0);         // UDP가 아닌 TCP로 패킷 손실 최소화
    av_dict_set(&opts, "max_delay",      "500000", 0);      // 최대 지연 0.5초
    av_dict_set(&opts, "buffer_size", "65536", 0);          // 버퍼 크기 증가
    av_dict_set(&opts, "stimeout", "5000000", 0);           // 5초 타임아웃
    ret = avformat_open_input(&formatContext, url, nullptr, &opts);
    if (ret < 0) return -1;

    av_dict_free(&opts);

    ret = avformat_find_stream_info(formatContext, nullptr);
    if (ret < 0) { avformat_close_input(&formatContext); return -1; }

    int data_stream_index = -1, video_stream_index = -1;
    for (unsigned int i = 0; i < formatContext->nb_streams; i++) {
        if (formatContext->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_DATA) {
            data_stream_index = i;
        } else if (formatContext->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            video_stream_index = i;
            if (!videoProcessor.initialize(formatContext->streams[i]->codecpar)) {
                avformat_close_input(&formatContext); return -1;
            }
        }
    }

    if (data_stream_index == -1 || video_stream_index == -1) {
        avformat_close_input(&formatContext); return -1;
    }

    std::thread syncThread(sync_thread);

    while (av_read_frame(formatContext, &pkt) >= 0 && !interrupted) {
        if (interrupted) break;
        if (pkt.stream_index == data_stream_index || pkt.stream_index == video_stream_index) {
            packet_count++;

            if (packet_count < 50) {
                std::cout << "[MAIN] Skipping packet for stabilization " << packet_count << std::endl;
                av_packet_unref(&pkt); // Don't forget to unref skipped packets
                continue;
            }

            AVPacket *cloned_pkt = av_packet_clone(&pkt);
            if (cloned_pkt) {
                std::lock_guard<std::mutex> lock(buf_mutex_);
                if (cloned_pkt->stream_index == data_stream_index) {
                    std::cout << "[MAIN] Received **metadata** packet PTS:" << cloned_pkt->pts << std::endl;
                    parser.processPacket(cloned_pkt);
                    
                    // Check if there are completed results before accessing
                    auto completed_results = parser.getCompletedResults();
                    if (!completed_results.empty()) {
                        metadata_results_.insert(metadata_results_.end(), 
                                               completed_results.begin(), completed_results.end());
                        std::cout << "[MAIN] Metadata results size: " << metadata_results_.size() << std::endl;
                    }
                    
                    // Clean up the cloned packet for metadata
                    av_packet_unref(cloned_pkt);
                    av_packet_free(&cloned_pkt);
                }
                else {
                    std::cout << "[MAIN] Received **video** packet PTS:" << cloned_pkt->pts << std::endl;
                    video_buf_.push_back(cloned_pkt); 
                }
                buf_cond_.notify_one();
            }
        }
        av_packet_unref(&pkt);
        
        // Check if interrupted during processing
        if (interrupted) {
            std::cout << "[MAIN] Interrupted during processing, breaking loop" << std::endl;
            break;
        }
    }

    std::cout << "[MAIN] Main loop ended, stopping sync thread..." << std::endl;
    stopSync = true;
    buf_cond_.notify_all();
    syncThread.join();

    videoProcessor.flush(); // Flush any remaining frames in VideoProcessor

    // Clean up OpenCV windows (only if GUI was available)
    try {
        cv::destroyAllWindows();
    } catch (...) {
        // Ignore GUI cleanup errors
    }

    avformat_close_input(&formatContext);
    avformat_network_deinit();
    return 0;
}