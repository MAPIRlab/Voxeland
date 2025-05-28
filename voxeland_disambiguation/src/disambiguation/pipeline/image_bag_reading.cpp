#include "disambiguation/pipeline/pipeline_steps.hpp"
#include "disambiguation/json_semantics.hpp"
#include <exception>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include "rosbag2_transport/reader_writer_factory.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <vector>

ImageBagReading::ImageBagReading(const std::string& bag_path){
    this->bag_path = bag_path;
    
    // Setup the bag reader
    rosbag2_storage::StorageOptions storage_options;
    storage_options.storage_id = "sqlite3";
    storage_options.uri = bag_path;
    this->reader = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
}

bool ImageBagReading::execute(){
    VXL_INFO("[IMAGE_BAG_READING] Reading images from bag file: {} ...", bag_path);
    
    std::vector<UncertainInstance>* uncertain_instances = context -> get_uncertain_instances();
    try {
        obtain_bag_images(*uncertain_instances);
    } catch (std::exception) {
        VXL_ERROR("[IMAGE_BAG_READING] Error reading images from bag file: {}", bag_path);
        return false;
    }
    
    VXL_INFO("[IMAGE_BAG_READING] All images read ");
    return true;
}

void ImageBagReading::obtain_bag_images(std::vector<UncertainInstance>& uncertain_instances){
    // Open the bag file
    reader->open(bag_path);
    
    // Read all images from bag file
    while (reader->has_next()){
        rosbag2_storage::SerializedBagMessageSharedPtr message = reader->read_next();
        
        if(message->topic_name != "camera/rgb"){
            continue;
        }
        
        //Desearialize the message
        rclcpp::SerializedMessage serialized_message(*message->serialized_data);
        sensor_msgs::msg::Image::SharedPtr ros_msg = std::make_shared<sensor_msgs::msg::Image>();
        image_serializer.deserialize_message(&serialized_message, ros_msg.get());
        
        // Check for every uncertain instance, if the image timestamp is included in the selected appearances
        for (UncertainInstance& instance : uncertain_instances){
            add_selected_images(ros_msg, instance);
        }
    }

    reader->close();
}

void ImageBagReading::add_selected_images(sensor_msgs::msg::Image::SharedPtr image_msg, UncertainInstance& instance){
    std::map<std::string, std::vector<cv_bridge::CvImagePtr>>* appearances = instance.get_selected_images();
    cv_bridge::CvImagePtr cv_ptr;
    // Lopp through all the selected appearances
    for(auto& [category, appearances_vector] : *instance.get_selected_appearances()){
        for(uint32_t timestamp : appearances_vector){
            // If the timestamp is included in the selected appearances,
            // the image is added to the selected images map
            if(timestamp == image_msg->header.stamp.sec){
                // Convert the image to a open_cv image
                cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
                (*appearances)[category].push_back(cv_ptr);
                VXL_INFO("Image {} added to instance: {} - Category: {}", image_msg->header.stamp.sec,instance.get_instance()->InstanceID, category);
            }
        }
    }
}

void ImageBagReading::show_all_images(std::vector<UncertainInstance>& uncertain_instances){
    for (UncertainInstance& instance : uncertain_instances){
        std::map<std::string, std::vector<cv_bridge::CvImagePtr>>* selected_images = instance.get_selected_images();
        for (auto& [category, images] : *selected_images){
            show_category_images(category,images);
        }
    }
}

void ImageBagReading::show_category_images(std::string category, std::vector<cv_bridge::CvImagePtr> images){
    for (cv_bridge::CvImagePtr image : images){
        cv::imshow(category, image->image);
        cv::waitKey(0);
    }
}