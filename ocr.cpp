#include "ocr.hpp"
#include <algorithm>
#include <numeric>
#include <cmath>

void TFOCR::save(const cv::Mat& img, const std::string& filename) {
    std::string full_path = preprocess_dir + "/" + filename;
    cv::imwrite(full_path, img);
}

std::vector<cv::Point2f> TFOCR::order_points(const std::vector<cv::Point>& pts) {
    std::vector<cv::Point2f> src(4);

    auto sum = [](const cv::Point2f& p) { return p.x + p.y; };
    auto diff = [](const cv::Point2f& p) { return p.y - p.x; };

    src[0] = *std::min_element(pts.begin(), pts.end(), [&](const cv::Point2f& a, const cv::Point2f& b) { return sum(a) < sum(b); }); // TL
    src[2] = *std::max_element(pts.begin(), pts.end(), [&](const cv::Point2f& a, const cv::Point2f& b) { return sum(a) < sum(b); }); // BR
    src[1] = *std::min_element(pts.begin(), pts.end(), [&](const cv::Point2f& a, const cv::Point2f& b) { return diff(a) < diff(b); }); // TR
    src[3] = *std::max_element(pts.begin(), pts.end(), [&](const cv::Point2f& a, const cv::Point2f& b) { return diff(a) < diff(b); }); // BL

    return src;
}


cv::Mat TFOCR::preprocess(const cv::Mat& input, float resize_factor) {
    cv::Mat gray, resized, binary, kernel;
    cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
    cv::resize(gray, resized, cv::Size(), resize_factor, resize_factor, cv::INTER_CUBIC);
    cv::threshold(resized, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    kernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
    cv::dilate(binary, binary, kernel, cv::Point(-1, -1), 1);
    return binary;
}

bool TFOCR::extract_plate_region(const cv::Mat& input, cv::Mat& output_plate) {
    cv::Mat hsv;
    cv::cvtColor(input, hsv, cv::COLOR_BGR2HSV);

    cv::Scalar lower_yellow(15, 100, 100);
    cv::Scalar upper_yellow(35, 255, 255);
    cv::Mat mask;
    cv::inRange(hsv, lower_yellow, upper_yellow, mask);

    cv::Mat masked;
    cv::bitwise_and(input, input, masked, mask);

    cv::Mat gray, blur, edges;
    cv::cvtColor(masked, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blur, cv::Size(5, 5), 0);
    cv::Canny(blur, edges, 50, 150);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<cv::Point> candidate;

    for (const auto& cnt : contours) {
        std::vector<cv::Point> approx;
        cv::approxPolyDP(cnt, approx, 0.02 * cv::arcLength(cnt, true), true);
        double area = cv::contourArea(approx);
        if (approx.size() == 4 && area > 500) {
            candidate = approx;
            break;
        }
    }

    if (candidate.empty()) {
        return false;
    }

    std::vector<cv::Point2f> corners = order_points(candidate);

    cv::Mat temp = input.clone();
    for (const auto& pt : corners)
        cv::circle(temp, pt, 5, cv::Scalar(0, 255, 0), -1);

    cv::Point2f tl = corners[0], tr = corners[1], br = corners[2], bl = corners[3];
    int width = static_cast<int>(std::max(cv::norm(br - bl), cv::norm(tr - tl)));
    int height = static_cast<int>(std::max(cv::norm(tr - br), cv::norm(tl - bl)));

    float fx = static_cast<float>(width - 1);
    float fy = static_cast<float>(height - 1);

    std::vector<cv::Point2f> dst_pts = {
        {0.0f, 0.0f},
        {fx, 0.0f},
        {fx, fy},
        {0.0f, fy}
    };

    cv::Mat M = cv::getPerspectiveTransform(corners, dst_pts);
    cv::warpPerspective(input, output_plate, M, cv::Size(width, height));

    return true;
}

cv::Mat TFOCR::preprocess_plate(const cv::Mat& input_img, int index) {
    cv::Mat plate_img_origin = input_img.clone();

    cv::Mat plate_img;
    if (!extract_plate_region(input_img, plate_img)) {
        return plate_img_origin;
    }

    return plate_img; // Return the processed plate image

}


std::map<int, std::string> TFOCR::loadLabelMap(const std::string& path) {
    std::map<int, std::string> label_map;
    std::ifstream file(path);
    std::string line;
    int idx = 0;
    while (std::getline(file, line)) {
        label_map[idx++] = line;
    }
    return label_map;
}

std::vector<int> TFOCR::ctcGreedyDecoder(const float* logits, int time, int classes) {
    std::vector<int> result;
    int prev = -1;
    for (int t = 0; t < time; ++t) {
        int max_index = 0;
        float max_val = logits[t * classes];
        for (int c = 1; c < classes; ++c) {
            float val = logits[t * classes + c];
            if (val > max_val) {
                max_val = val;
                max_index = c;
            }
        }
        if (max_index != prev && max_index != 0) {
            result.push_back(max_index);
        }
        prev = max_index;
    }
    return result;
}

float TFOCR::getConfidence(const float* logits, int time, int classes, const std::string& mode) {
    std::vector<float> confidences;
    int blank = 0; // blank token index
    
    for (int t = 0; t < time; ++t) {
        // Find max index and value for this timestep
        int max_index = 0;
        float max_val = logits[t * classes];
        for (int c = 1; c < classes; ++c) {
            float val = logits[t * classes + c];
            if (val > max_val) {
                max_val = val;
                max_index = c;
            }
        }
        
        // Skip blank tokens
        if (max_index == blank) continue;
        
        // Apply exponential (assuming logits are log probabilities)
        float confidence = std::exp(max_val);
        confidences.push_back(confidence);
    }
    
    if (confidences.empty()) {
        return 0.0f;
    }
    
    if (mode == "min") {
        return *std::min_element(confidences.begin(), confidences.end());
    } else {
        // mean mode
        float sum = std::accumulate(confidences.begin(), confidences.end(), 0.0f);
        return sum / confidences.size();
    }
}

void TFOCR::load_ocr(const std::string& model_path, const std::string& labels_path) {
    // Load label map
    label_map = loadLabelMap(labels_path);

    // Load TFLite model
    model = tflite::FlatBufferModel::BuildFromFile(model_path.c_str());
    tflite::ops::builtin::BuiltinOpResolver resolver;
    // Removed local interpreter declaration, now using member variable
    tflite::InterpreterBuilder(*model, resolver)(&interpreter);
    interpreter->AllocateTensors();
}

std::string TFOCR::removeRegionalName(const std::string& text) {
    if (text.length() < 6) { // Need at least 2 Korean characters (6 bytes in UTF-8)
        return text;
    }
    
    // List of Korean regional names (2 characters each in UTF-8)
    std::vector<std::string> regional_names = {
        "서울", "부산", "대구", "인천", "광주", "대전", "울산", 
        "세종", "경기", "강원", "충북", "충남", "전북", "전남", 
        "경북", "경남", "제주"
    };
    
    // Extract first 6 bytes (2 Korean characters)
    std::string first_two_chars = text.substr(0, 6);
    
    // Check if it matches any regional name
    for (const auto& region : regional_names) {
        if (first_two_chars == region) {
            // Remove the regional name and return the rest
            return text.substr(6);
        }
    }
    
    // If no regional name found, return original text
    return text;
}

TFOCR::OCRResult TFOCR::run_ocr(const cv::Mat& input_img) {

    cv::Mat gray = preprocess_plate(input_img, 0); // Preprocess the input image
    cv::cvtColor(input_img, gray, cv::COLOR_BGR2GRAY);
    cv::resize(gray, gray, cv::Size(192, 96));
    cv::imwrite("debug_ocr_input.jpg", gray); // Debugging line
    gray.convertTo(gray, CV_32FC1, 1.0 / 255.0);

    // Set input
    float* input = interpreter->typed_input_tensor<float>(0);
    memcpy(input, gray.data, 96 * 192 * sizeof(float));

    // Run inference
    if (interpreter->Invoke() != kTfLiteOk) {
        std::cerr << "ocr inference failed..." << std::endl;
        return {"", 0.0f};
    }

    // Decode output
    auto output_details = interpreter->tensor(interpreter->outputs()[0]);
    const float* output_data = output_details->data.f;
    int time = output_details->dims->data[1];
    int classes = output_details->dims->data[2];

    std::vector<int> indices = ctcGreedyDecoder(output_data, time, classes);

    // Convert indices to characters
    std::string result;
    for (int idx : indices) {
        if (label_map.count(idx)) {
            result += label_map[idx];
        }
    }

    // Remove regional name if present
    result = removeRegionalName(result);


    // Calculate confidence
    float confidence = getConfidence(output_data, time, classes, "min");
    confidence = std::round(confidence * 10000.0f) / 10000.0f; // Round to 4 decimal places

    // Return empty result if confidence is too low
    if (confidence < min_confidence_threshold) {
        // std::cout << "[OCR] Low confidence (" << confidence << " < " << min_confidence_threshold 
                //   << "), returning empty result" << std::endl;
        return {"", 0.0f};
    }

    return {result, confidence};
}