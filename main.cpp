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

#include "parser.hpp"
#include "video.hpp"

int main() {
    avformat_network_init(); // Initialize

    MetadataParser parser; // MetadataParser
    VideoProcessor videoProcessor; // VideoProcessor
    AVFormatContext *formatContext = nullptr;
    AVPacket pkt;
    AVDictionary *options = nullptr;

    int ret = -1;

    // RTSP options setup
    av_dict_set(&options, "rtsp_transport", "tcp", 0);  // TCP 
    av_dict_set(&options, "rtsp_flags", "prefer_tcp", 0);
    av_dict_set(&options, "buffer_size", "1024000", 0); // 1MB buffer
    av_dict_set(&options, "max_delay", "500000", 0);    // 0.5s maxima delay
    av_dict_set(&options, "timeout", "5000000", 0);     // 5s timeout

    const char* url = "rtsp://192.168.0.64/profile2/media.smp";
    std::cout << "Connecting to RTSP stream: " << url << std::endl;
    
    ret = avformat_open_input(&formatContext, url, nullptr, &options);
    av_dict_free(&options); 
    if (ret < 0) {
        fprintf(stderr, "Could not open input: %s\n", av_err2str(ret));
        return -1;  
    }
    ret = avformat_find_stream_info(formatContext, nullptr);
    if (ret < 0) {        
        fprintf(stderr, "Could not find stream info: %s\n", av_err2str(ret));
        avformat_close_input(&formatContext);
        return -1;
    }

    int data_stream_index = -1;
    int video_stream_index = -1;
    
    for (unsigned int i=0; i < formatContext->nb_streams; i++) {
        AVStream *stream = formatContext->streams[i];
        if (stream->codecpar->codec_type == AVMEDIA_TYPE_DATA) {
            data_stream_index = i;
            std::cout << "Found data stream at index: " << data_stream_index << std::endl;
        }
        else if (stream->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            video_stream_index = i;
            std::cout << "Found video stream at index: " << video_stream_index << std::endl;
            std::cout << "Video codec: " << avcodec_get_name(stream->codecpar->codec_id) << std::endl;
            std::cout << "Video resolution: " << stream->codecpar->width << "x" << stream->codecpar->height << std::endl;
            
            // Initialize video processor
            if (!videoProcessor.initialize(stream->codecpar)) {
                std::cerr << "Failed to initialize video processor" << std::endl;
            }
        }
    }

    if (data_stream_index < 0) {
        fprintf(stderr, "No data stream found\n");
    }
    if (video_stream_index < 0) {
        fprintf(stderr, "No video stream found\n");
    }
    if (data_stream_index < 0 && video_stream_index < 0) {
        avformat_close_input(&formatContext);
        return -1;
    }

    int packet_count = 0;
    int video_packet_count = 0;
    int data_packet_count = 0;
    auto start_time = std::chrono::steady_clock::now();

    while(av_read_frame(formatContext, &pkt) >= 0) {
        packet_count++;
        
        if (pkt.stream_index == data_stream_index) {
            data_packet_count++;
            std::cout << "Processing data packet #" << data_packet_count << " (size: " << pkt.size << " bytes)" << std::endl;
            std::cout << "Packet PTS: " << pkt.pts << ", DTS: " << pkt.dts << std::endl;
            parser.processPacket(pkt.data, pkt.size); // Process the metadata packet
            std::vector<Object> result = parser.getResults(); // Get the results from the parser
            
            // Send detection results to video processor
            if (videoProcessor.isInitialized() && !result.empty()) {
                // Only DTS is valid
                if (pkt.dts != AV_NOPTS_VALUE) {
                    videoProcessor.setDetectionResults(result, pkt.dts);
                    std::cout << "Updated video processor with " << result.size() << " detection results (DTS: " << pkt.dts << ")" << std::endl;
                } else {
                    std::cout << "Skipping detection results due to invalid DTS" << std::endl;
                }
            }
            
            // for (const auto& item : result) {
            //     std::cout << "  Type: " << item.typeName << "  ID: " << item.objectId << ", Confidence: " << item.confidence << std::endl;
            //     std::cout << "    BoundingBox: [" << item.boundingBox.left << ", " << item.boundingBox.top 
            //               << ", " << item.boundingBox.right << ", " << item.boundingBox.bottom << "]" << std::endl;
            //     std::cout << "    Center: (" << item.centerOfGravity.x << ", " << item.centerOfGravity.y << ")" << std::endl;
            // }
        }
        else if (pkt.stream_index == video_stream_index) {
            video_packet_count++;
            std::cout << "Processing video packet #" << video_packet_count << " (size: " << pkt.size << " bytes)" << std::endl;
            std::cout << "Packet PTS: " << pkt.pts << ", DTS: " << pkt.dts << std::endl;
            // Process video packet with OpenCV display
            if (videoProcessor.isInitialized()) {
                videoProcessor.processPacket(&pkt, pkt.dts);
            }
        }
        
        av_packet_unref(&pkt); // Free the packet
    }
    
    parser.processBuffer(); // Process any remaining buffered data
    avformat_close_input(&formatContext); // Close the input

    return 0; // Exit successfully

}