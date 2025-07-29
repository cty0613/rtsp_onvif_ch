#include "parser.hpp"
#include <vector>

using namespace tinyxml2;

std::vector<Object> MetadataParser::extractObj(XMLElement* root) {
    std::vector<Object> objItems;
    if (!root) return objItems;

    XMLElement* analytics = root->FirstChildElement("tt:VideoAnalytics");
    if (!analytics) return objItems;

    XMLElement* frame = analytics->FirstChildElement("tt:Frame");
    if (!frame) return objItems;

    for (XMLElement* obj = frame->FirstChildElement("tt:Object"); obj; obj = obj->NextSiblingElement("tt:Object")) {
        const char* objectId = obj->Attribute("ObjectId");
        if (!objectId) continue;

        const char* Lp = "LicensePlate";
        Object objItem;
        objItem.typeName = "Unknown";
        objItem.objectId = std::stoi(objectId);
        objItem.confidence = 0.0;
        objItem.boundingBox = {0.0, 0.0, 0.0, 0.0};
        objItem.centerOfGravity = {0.0, 0.0};

        XMLElement* appearance = obj->FirstChildElement("tt:Appearance");
        if (appearance) {
            XMLElement* shape = appearance->FirstChildElement("tt:Shape");
            if (shape) {
                XMLElement* bbox = shape->FirstChildElement("tt:BoundingBox");
                if (bbox) {
                    bbox->QueryDoubleAttribute("left", &objItem.boundingBox.left);
                    bbox->QueryDoubleAttribute("top", &objItem.boundingBox.top);
                    bbox->QueryDoubleAttribute("right", &objItem.boundingBox.right);
                    bbox->QueryDoubleAttribute("bottom", &objItem.boundingBox.bottom);
                }
                XMLElement* cog = shape->FirstChildElement("tt:CenterOfGravity");
                if (cog) {
                    cog->QueryDoubleAttribute("x", &objItem.centerOfGravity.x);
                    cog->QueryDoubleAttribute("y", &objItem.centerOfGravity.y);
                }
            }

            XMLElement* classElem = appearance->FirstChildElement("tt:Class");
            if (classElem) {
                XMLElement* type = classElem->FirstChildElement("tt:Type");
                if (type) {
                    // Only process LicensePlate type objects
                    if (strcmp(type->GetText(), Lp) == 0) {
                        objItem.typeName = type->GetText();
                        type->QueryDoubleAttribute("Likelihood", &objItem.confidence);
                        objItems.push_back(objItem);
                    }
                }
            }
        }
    }
    return objItems;
}

void MetadataParser::processXmlDoc(XMLDocument& doc, std::vector<Object>& result) {
    XMLElement* root = doc.RootElement();
    if (!root) {
        std::cerr << "No root element found" << std::endl;
        return;
    }

    std::vector<Object> detections = extractObj(root);
    if (!detections.empty()) {
        result.insert(result.end(), detections.begin(), detections.end());
    }
}

void MetadataParser::processPacket(AVPacket* packet) {
    if (!packet || !packet->data || packet->size <= 0) return;

    xml_buffer.append(reinterpret_cast<const char*>(packet->data), packet->size);
    processBuffer(packet->pts);
}

void MetadataParser::processBuffer(int64_t pts) {
    size_t pos = 0;
    const std::string xml_declaration = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>";

    while ((pos = xml_buffer.find("<tt:MetadataStream", pos)) != std::string::npos) {
        size_t end_pos = xml_buffer.find("</tt:MetadataStream>", pos);
        if (end_pos == std::string::npos) {
            // Not a complete element, break and wait for more data
            break;
        }
        size_t element_end = end_pos + std::string("</tt:MetadataStream>").length();
        std::string complete_element = xml_buffer.substr(pos, element_end - pos);
        
        // Store the complete XML string with its PTS
        completed_streams.emplace_back(xml_declaration + complete_element, pts);
        
        // Erase the processed part from the buffer
        xml_buffer.erase(0, element_end);
        pos = 0; // Reset position to search from the beginning of the modified buffer
    }
    processCompletedStreams();
    cleanupBuffer();
}

void MetadataParser::processCompletedStreams() {
    if (completed_streams.empty()) return;

    for (const auto& stream_pair : completed_streams) {
        const std::string& xmlString = stream_pair.first;
        int64_t pts = stream_pair.second;

        XMLDocument doc;
        if (doc.Parse(xmlString.c_str()) == XML_SUCCESS) {
            std::vector<Object> objects;
            processXmlDoc(doc, objects);
            if (!objects.empty()) {
                results.push_back({pts, objects});
            }
        } else {
            std::cerr << "Failed to parse XML: " << doc.ErrorStr() << std::endl;
        }
    }
    completed_streams.clear();
}

std::vector<MetadataResult> MetadataParser::getCompletedResults() {
    std::vector<MetadataResult> completed = std::move(results);
    results.clear();
    return completed;
}

void MetadataParser::cleanupBuffer() {
    const size_t MAX_BUFFER_SIZE = 512 * 1024; // 512KB
    if (xml_buffer.size() > MAX_BUFFER_SIZE) {
        size_t next_metadata = xml_buffer.find("<tt:MetadataStream");
        if (next_metadata != std::string::npos) {
            xml_buffer.erase(0, next_metadata);
        } else {
            // If no start tag is found, keep the end of the buffer
            xml_buffer.erase(0, xml_buffer.size() - 1024); 
        }
    }
}

size_t MetadataParser::getBufferSize() const {
    return xml_buffer.size();
}

void MetadataParser::debugBuffer() {
    // Implementation of debugBuffer can be added here if needed
}

