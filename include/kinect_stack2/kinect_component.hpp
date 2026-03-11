#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

extern "C" {
    #include <libfreenect.h>
}

#include <camera_info_manager/camera_info_manager.hpp>

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>


class KinectNode : public rclcpp::Node {
public:
    KinectNode(const rclcpp::NodeOptions&);
    ~KinectNode();

private:
    void timer_callback();
    static void video_cb(freenect_device *dev, void *video, uint32_t /*timestamp*/);
    static void depth_cb(freenect_device *dev, void *depth, uint32_t /*timestamp*/);

    freenect_context *f_ctx_{nullptr};
    freenect_device *dev_{nullptr};

    image_transport::CameraPublisher rgb_pub_, depth_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr tilt_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr tilt_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr led_sub_;

    std::shared_ptr<camera_info_manager::CameraInfoManager> rgb_info_manager_, depth_info_manager_;
    sensor_msgs::msg::CameraInfo rgb_info_, depth_info_;

    std::thread kinect_thread_;
    rclcpp::TimerBase::SharedPtr timer_;
};