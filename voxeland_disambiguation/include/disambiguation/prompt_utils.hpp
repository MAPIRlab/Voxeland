
#include <fmt/format.h>
#include <fmt/ranges.h>  // Necesario para fmt::join
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>
#include <fstream>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"


std::string base64_encode_(const std::vector<unsigned char>& data) {
    const std::string base64_chars =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789+/";
    
    std::string encoded;
    size_t len = data.size();
    encoded.reserve(((len + 2) / 3) * 4);  // Reservar espacio para optimizar
    
    for (size_t i = 0; i < len; ++i) {
        uint32_t triplet = 0;
        int bytes_in_triplet = 0;

        // Cargar hasta 3 bytes
        triplet |= static_cast<uint32_t>(data[i]) << 16;
        bytes_in_triplet++;
        
        if (++i < len) {
            triplet |= static_cast<uint32_t>(data[i]) << 8;
            bytes_in_triplet++;
        }
        
        if (++i < len) {
            triplet |= static_cast<uint32_t>(data[i]);
            bytes_in_triplet++;
        }

        // Dividir en 4 índices de 6 bits
        encoded += base64_chars[(triplet >> 18) & 0x3F];
        encoded += base64_chars[(triplet >> 12) & 0x3F];

        if (bytes_in_triplet > 1) {
            encoded += base64_chars[(triplet >> 6) & 0x3F];
        } else {
            encoded += '=';
        }

        if (bytes_in_triplet > 2) {
            encoded += base64_chars[triplet & 0x3F];
        } else {
            encoded += '=';
        }
    }
    
    return encoded;
}

std::string image_to_base64(cv_bridge::CvImagePtr image){
    std::vector<uchar> buf;
    cv::imencode(".png", image->image, buf);
    std::string base64_image = base64_encode_(buf);
    return base64_image;
}

const std::string load_and_format_prompt(const std::string& file_name, std::vector<std::string> categories, int images_per_instance){
    std::string package_path = ament_index_cpp::get_package_share_directory("voxeland_disambiguation");
    std::string prompt_file_path = package_path + "/params/" + file_name;

    int total_images = categories.size() * images_per_instance;

    std::string res_prompt;
    std::string line;
    std::ifstream prompt_file(prompt_file_path);    
    while(std::getline(prompt_file, line)){
        int pos = line.find("{}");

        if (pos != std::string::npos){
             // If the line constains the placeholder, replace it with the categories vector
            res_prompt += line.substr(0, pos + 1);
            for(auto it = categories.begin(); it != categories.end(); ++it){
                res_prompt += *it;
                if (it != categories.end() - 1){
                    res_prompt += ",";
                }
            }
            res_prompt += line.substr(pos + 1);
        } else {
            res_prompt += line;
        }
        res_prompt += "\n";
    }
    // Add <image> tags
    // for(int i = 0; i < total_images; i++){
    //     res_prompt += "<image>";
    // }


    return res_prompt;
}


