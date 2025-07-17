#include "parser.hpp"

using namespace tinyxml2;


std::string MetadataParser::extractTime(XMLElement* element) {
    if (!element) return "";
        
    // VideoAnalytics 내부의 Frame에서 시간 찾기
    XMLElement* analytics = element->FirstChildElement("tt:VideoAnalytics");
    if (analytics) {
        XMLElement* frame = analytics->FirstChildElement("tt:Frame");
        if (frame) {
            const char* utcTime = frame->Attribute("UtcTime");
            if (utcTime) return utcTime;
        }
    }
    
        
    return "";
}

std::vector<Object> MetadataParser::extractObj(XMLElement* root) {
    std::vector<Object> objItems;
        
    if (!root) return objItems;
    
    // find tt:VideoAnalytics from Root Element
    XMLElement* analytics = root->FirstChildElement("tt:VideoAnalytics");
    if (!analytics) return objItems;
    
    // find tt:Frame from tt:VideoAnalytics
    XMLElement* frame = analytics->FirstChildElement("tt:Frame");
    if (!frame) return objItems;
    
    // 각 객체를 순회
    for (XMLElement* obj = frame->FirstChildElement("tt:Object"); obj; 
            obj = obj->NextSiblingElement("tt:Object")) {
            
        const char* objectId = obj->Attribute("ObjectId");
        if (!objectId) continue;
        
        Object objItem;
        objItem.typeName = "Unknown";
        objItem.objectId = std::stoi(objectId);
        objItem.confidence = 0.0;
        objItem.boundingBox = {0.0, 0.0, 0.0, 0.0};
        objItem.centerOfGravity = {0.0, 0.0};
        
        XMLElement* appearance = obj->FirstChildElement("tt:Appearance");
        if (appearance) {
            // Extract Shape information
            XMLElement* shape = appearance->FirstChildElement("tt:Shape");
            if (shape) {
                // Extract BoundingBox
                XMLElement* bbox = shape->FirstChildElement("tt:BoundingBox");
                if (bbox) {
                    const char* left = bbox->Attribute("left");
                    const char* top = bbox->Attribute("top");
                    const char* right = bbox->Attribute("right");
                    const char* bottom = bbox->Attribute("bottom");
                    
                    if (left && top && right && bottom) {
                        objItem.boundingBox.left = std::stod(left);
                        objItem.boundingBox.top = std::stod(top);
                        objItem.boundingBox.right = std::stod(right);
                        objItem.boundingBox.bottom = std::stod(bottom);
                    }
                }
                
                // Extract CenterOfGravity
                XMLElement* cog = shape->FirstChildElement("tt:CenterOfGravity");
                if (cog) {
                    const char* x = cog->Attribute("x");
                    const char* y = cog->Attribute("y");
                    
                    if (x && y) {
                        objItem.centerOfGravity.x = std::stod(x);
                        objItem.centerOfGravity.y = std::stod(y);
                    }
                }
            }
            
            // Extract Class information
            XMLElement* classElem = appearance->FirstChildElement("tt:Class");
            if (classElem) {
                XMLElement* type = classElem->FirstChildElement("tt:Type");
                if (type) {
                    const char* typeName = type->GetText();
                    const char* likelihood = type->Attribute("Likelihood");
                    
                    if (typeName && likelihood) {
                        objItem.confidence = std::stod(likelihood);
                        objItem.typeName = typeName; // Set the type name
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
    
    std::cout << "Root Node: " << root->Name() << std::endl;
    
    // 자식 요소가 있는지 확인
    bool hasChildren = root->FirstChildElement() != nullptr;
    if (!hasChildren) {
        std::cout << "Empty metadata document (no child elements)" << std::endl;
        return;
    }
    
    // 시간 정보 추출
    std::string time = extractTime(root);
    if (!time.empty()) {
        std::cout << "Timestamp: " << time << std::endl;
    }
    
    // 인간 객체 정보 추출
    std::vector<Object> detections = extractObj(root);
    if (!detections.empty()) {
        std::cout << "Detected " << detections.size() << " Object(s):" << std::endl;
        // for (const auto& item : detections) {
        //     std::cout << "  Type: " << item.typeName << "  ID: " << item.objectId << ", Confidence: " << item.confidence << std::endl;
        //     std::cout << "    BoundingBox: [" << item.boundingBox.left << ", " << item.boundingBox.top 
        //               << ", " << item.boundingBox.right << ", " << item.boundingBox.bottom << "]" << std::endl;
        //     std::cout << "    Center: (" << item.centerOfGravity.x << ", " << item.centerOfGravity.y << ")" << std::endl;
        // }
        result = detections; // Store the results in the provided vector
    }
    
}

void MetadataParser::processPacket(const uint8_t* data, int size) {
    
    if (!data || size <= 0) return;
    
    // 버퍼에 데이터 추가
    xml_buffer.append(reinterpret_cast<const char*>(data), size);
    
    // XML 문서 완성 여부 확인 및 처리
    processBuffer();
}

void MetadataParser::processBuffer() {
    size_t pos = 0;
    const std::string xml_declaration = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>";

    while (pos < xml_buffer.size()) {

        // find <tt:MetadataStream start tag
        size_t start_pos = xml_buffer.find("<tt:MetadataStream", pos);
        if (start_pos == std::string::npos) {
            break;
        }
        
        size_t tag_end = xml_buffer.find(">", start_pos); // find '>'
        if (tag_end == std::string::npos) {
            break;
        }
        
        std::string start_tag = xml_buffer.substr(start_pos, tag_end - start_pos + 1);  // exception absolute <tt:MetadataStream /> 
        if (start_tag.find("/>") != std::string::npos) {
            completed_streams.push_back(xml_declaration + start_tag);
            pos = tag_end + 1;
            continue;
        }

        // find </tt:MetadataStream end tag
        size_t end_pos = xml_buffer.find("</tt:MetadataStream>", tag_end);
        if (end_pos == std::string::npos) {
            break; // end tag not found, wait for more data
        } else {
            size_t element_end = end_pos + std::string("</tt:MetadataStream>").length();
            std::string complete_element = xml_buffer.substr(start_pos, element_end - start_pos);
            completed_streams.push_back(xml_declaration + complete_element);

            xml_buffer.erase(0, element_end); // remove processed element from buffer
            pos = 0; // reset position for next search
        }

    }
    
    // Process completed XML streams
    processCompletedStreams();
    
    // debugBuffer(); // debug current buffer state
    cleanupBuffer(); // clean up buffer after processing
}

void MetadataParser::processCompletedStreams() {
    for (const std::string& xmlString : completed_streams) {
        XMLDocument doc;
        XMLError par_result = doc.Parse(xmlString.c_str());
        
        if (par_result == XML_SUCCESS) {
            processXmlDoc(doc, result);
        } else {
            std::cerr << "Failed to parse XML: " << doc.ErrorStr() << std::endl;
        }
    }
    
    // Clear processed streams
    completed_streams.clear();
}

void MetadataParser::cleanupBuffer() {
    // MetadataStream 태그가 아닌 쓰레기 데이터 제거
        const std::string pattern = "<tt:MetadataStream";
        size_t next_metadata = xml_buffer.find(pattern);
        
        if (next_metadata != std::string::npos && next_metadata > 0) {
            // MetadataStream 이전의 데이터 제거
            xml_buffer.erase(0, next_metadata);
        } else if (next_metadata == std::string::npos) {
            // MetadataStream 태그가 없으면 버퍼의 대부분 제거 (마지막 일부만 보존)
            const size_t KEEP_SIZE = 100; // 다음 패킷에서 완성될 수 있는 부분
            if (xml_buffer.size() > KEEP_SIZE) {
                xml_buffer.erase(0, xml_buffer.size() - KEEP_SIZE);
            }
        }
        
        // 버퍼 크기 제한
        const size_t MAX_BUFFER_SIZE = 512 * 1024; // 512KB
        if (xml_buffer.size() > MAX_BUFFER_SIZE) {
            xml_buffer.erase(0, xml_buffer.size() - MAX_BUFFER_SIZE / 2);
        }
}


size_t MetadataParser::getBufferSize() const {
    return xml_buffer.size();
}

void MetadataParser::debugBuffer() {
    std::cout << "=== MetadataParser Debug Information ===" << std::endl;
    std::cout << "Buffer size: " << xml_buffer.size() << " bytes" << std::endl;
    std::cout << "Completed streams count: " << completed_streams.size() << std::endl;
    
    if (!completed_streams.empty()) {
        std::cout << "\n--- Completed Streams ---" << std::endl;
        for (size_t i = 0; i < completed_streams.size(); ++i) {
            std::cout << "Stream " << (i + 1) << ":" << std::endl;
            std::cout << completed_streams[i] << std::endl;
            std::cout << "Length: " << completed_streams[i].length() << " bytes" << std::endl;
            std::cout << "---" << std::endl;
        }
    }
    
    if (!xml_buffer.empty()) {
        std::cout << "\n--- Current Buffer Content ---" << std::endl;
        std::cout << "First 200 chars: " << xml_buffer.substr(0, 200) << std::endl;
        if (xml_buffer.size() > 200) {
            std::cout << "... (truncated)" << std::endl;
        }
    }
    
    std::cout << "=======================================" << std::endl;
}

const std::vector<Object>& MetadataParser::getResults() const {
    return result;
}
