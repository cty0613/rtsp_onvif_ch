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

MetadataParser metadataParser;
VideoProcessor videoProcessor;


std::queue<AVPacket*> packet_queue;
std::queue<AVFrame*> frame_queue;


std::mutex packet_mutex;
std::mutex frame_mutex;
std::condition_variable packet_cond;
std::condition_variable frame_cond;


std::atomic<bool> stream_run{true};
std::atomic<bool> decode_run{true};
std::atomic<bool> sync_run{true};

std::atomic<bool> interrupted{false};

void stream_thread(AVFormatContext* formatContext, int data_stream_index, int video_stream_index) {

    AVPacket pkt;

    while(stream_run) {
        if (av_read_frame(formatContext, &pkt) >= 0) {
            if (pkt.stream_index == data_stream_index || pkt.stream_index == video_stream_index) {
                AVPacket *cloned_pkt = av_packet_clone(&pkt);
                std::unique_lock<std::mutex> lock(packet_mutex);
                packet_queue.push(cloned_pkt);

                std::cout << "[STREAM] Received packet PTS: " << cloned_pkt->pts << " from stream index: " << pkt.stream_index << std::endl;
                av_packet_unref(cloned_pkt);
                av_packet_free(&cloned_pkt);
            }
            av_packet_unref(&pkt);
        } else {
            break; // Exit on read error
        } 
        packet_cond.notify_one();
    }


}

void decode_thread(AVFormatContext* formatContext, int data_stream_index, int video_stream_index) {

    AVCodecParameters* vid_codecpar = formatContext->streams[video_stream_index]->codecpar;
    AVCodec* codec;
    AVCodecContext* codec_context = nullptr;

    codec = avcodec_find_decoder(vid_codecpar->codec_id);
    if (!codec) {
        std::cerr << "Unsupported codec!" << std::endl;
        return;
    }

    codec_context = avcodec_alloc_context3(codec);
    if (!codec_context) {
        std::cerr << "Could not allocate video codec context" << std::endl;
        return;
    }

    if (avcodec_parameters_to_context(codec_context, vid_codecpar) < 0) {
        std::cerr << "Could not copy codec parameters to context" << std::endl;
        return;
    }

    if (avcodec_open2(codec_context, codec, nullptr) < 0) {
        std::cerr << "Could not open codec" << std::endl;
        return;
    }

    int width_ = codec_context->width;
    int height_ = codec_context->height;

    AVFrame* frame= av_frame_alloc();
    if (!frame) {
        std::cerr << "Could not allocate frame" << std::endl;
        return;
    }

    // Check pixel format and set default if needed
    if (codec_context->pix_fmt == AV_PIX_FMT_NONE) {
        std::cout << "Pixel format not set, using YUV420P as default" << std::endl;
        codec_context->pix_fmt = AV_PIX_FMT_YUV420P;
    }
    
    // Handle YUVJ420P (full range) properly
    AVPixelFormat target_pix_fmt = codec_context->pix_fmt;
    if (codec_context->pix_fmt == AV_PIX_FMT_YUVJ420P) {
        std::cout << "Converting YUVJ420P to YUV420P with full range" << std::endl;
        target_pix_fmt = AV_PIX_FMT_YUV420P;
    }

    SwsContext* sws_ctx = sws_getContext(
        width_, height_, target_pix_fmt,  // 입력 포맷
        width_, height_, AV_PIX_FMT_BGR24,         // 출력 포맷
        SWS_LANCZOS | SWS_FULL_CHR_H_INT | SWS_FULL_CHR_H_INP | SWS_ACCURATE_RND, // 고품질 보간
        nullptr, nullptr, nullptr
    );

    // Set color range and space for proper conversion
    if (codec_context->pix_fmt == AV_PIX_FMT_YUVJ420P) {
        const int *inv_table, *table;
        int srcRange, dstRange, brightness, contrast, saturation;
        
        inv_table = sws_getCoefficients(SWS_CS_ITU709);
        sws_setColorspaceDetails(sws_ctx, inv_table, 1, // src: full range
                               inv_table, 0,             // dst: limited range  
                               0, 1 << 16, 1 << 16);     // brightness, contrast, saturation
    }

    if (!sws_ctx) {
        std::cerr << "Could not initialize SwsContext" << std::endl;
        std::cerr << "Input format: " << av_get_pix_fmt_name(codec_context->pix_fmt) << std::endl;
        std::cerr << "Target format: " << av_get_pix_fmt_name(target_pix_fmt) << std::endl;
        std::cerr << "Dimensions: " << width_ << "x" << height_ << std::endl;
        return;
    }

    AVPacket* pkt = nullptr;
    while (decode_run) {
        pkt = packet_queue.pop();
        avcodec_send_packet(codec_context, pkt);
        std::cout << "[DECODE] Processing packet PTS: " << pkt->pts << std::endl;
        while (avcodec_receive_frame(codec_context, frame)) {
                frame_queue.push(frame);
        }
    }

}

void sync_thread() {

}


int main() {

    const char* url = "rtsp://192.168.0.64/profile2/media.smp";

    avformat_network_init();
    AVFormatContext *formatContext = nullptr;

    AVDictionary *opts = nullptr;
    av_dict_set(&opts, "rtsp_transport", "tcp", 0);         // UDP가 아닌 TCP로 패킷 손실 최소화
    av_dict_set(&opts, "max_delay",      "500000", 0);      // 최대 지연 0.5초
    av_dict_set(&opts, "buffer_size", "65536", 0);          // 버퍼 크기 증가
    av_dict_set(&opts, "stimeout", "5000000", 0);           // 5초 타임아웃

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
        } else if (formatContext->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            video_stream_index = i;
            if (!videoProcessor.initialize(formatContext->streams[i]->codecpar)) {
                avformat_close_input(&formatContext);
                return -1;
            }
        }
    }

    if (data_stream_index == -1 || video_stream_index == -1) {
        avformat_close_input(&formatContext); return -1;
    }

    std::thread streamThread(stream_thread, std::ref(formatContext), data_stream_index, video_stream_index);
    std::thread decodeThread(decode_thread, std::ref(formatContext), data_stream_index, video_stream_index);
    std::thread syncThread(sync_thread);

    stream_run = false;
    decode_run = false;
    sync_run = false;

    streamThread.join();
    decodeThread.join();
    syncThread.join();



    return 0;
}