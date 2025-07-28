#ifndef PARSER_HPP
#define PARSER_HPP

#include <tinyxml2.h>
#include <iostream>
#include <algorithm>
#include <string>
#include <mutex>
#include <vector>
#include <cstdint>

extern "C" {
#include <libavcodec/avcodec.h>
}

struct BoundingBox {
    double left, top, right, bottom;
};

struct Point {
    double x, y;
};

struct Object {
    std::string typeName;
    int objectId;
    double confidence;
    BoundingBox boundingBox;
    Point centerOfGravity;
};

struct MetadataResult {
    int64_t pts;
    std::vector<Object> objects;
};

class MetadataParser {
private:
    std::string xml_buffer;
    std::vector<std::pair<std::string, int64_t>> completed_streams;
    std::vector<MetadataResult> results;

    std::vector<Object> extractObj(tinyxml2::XMLElement* element);
    void processXmlDoc(tinyxml2::XMLDocument& doc, std::vector<Object>& result);
    void cleanupBuffer();
    void processCompletedStreams();

public:
    void processPacket(AVPacket* packet);
    void processBuffer(int64_t pts);
    size_t getBufferSize() const;
    void debugBuffer();
    std::vector<MetadataResult> getCompletedResults();
};

#endif // PARSER_HPP