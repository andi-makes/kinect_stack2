#include "kinect_stack2/kinect_component.hpp"

#include <sensor_msgs/image_encodings.hpp>

#include <chrono>
using namespace std::chrono_literals;

KinectNode::KinectNode(const rclcpp::NodeOptions& options)
: Node("kinect_node", options) {
    this->declare_parameter("rgb_frame", "kinect_rgb_optical_frame");
    this->declare_parameter("depth_frame", "kinect_depth_optical_frame");

    this->depth_pub_ = image_transport::create_camera_publisher(this, "depth/image_raw");
    this->rgb_pub_ = image_transport::create_camera_publisher(this, "rgb/image_raw");
    this->tilt_pub_ = create_publisher<std_msgs::msg::Float64>("current_tilt_degree", 10);
    this->timer_ = this->create_wall_timer(500ms, std::bind(&KinectNode::timer_callback, this));

    std::string pkg_share = ament_index_cpp::get_package_share_directory("kinect_stack2");

    depth_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
        this, "kinect",
        "file://" + pkg_share + "/config/calibration_depth.yaml");

    rgb_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
        this, "kinect",
        "file://" + pkg_share + "/config/calibration_rgb.yaml");

    rgb_info_ = rgb_info_manager_->getCameraInfo();
    rgb_info_.header.frame_id = this->get_parameter("rgb_frame").as_string();

    depth_info_ = depth_info_manager_->getCameraInfo();
    depth_info_.header.frame_id = this->get_parameter("depth_frame").as_string();
        
    tilt_sub_ = create_subscription<std_msgs::msg::Float64>(
        "set_tilt_degree", 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
            freenect_set_tilt_degs(dev_, msg->data);
        }
    );

    led_sub_ = create_subscription<std_msgs::msg::Int32>(
        "led", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            freenect_set_led(dev_, static_cast<freenect_led_options>(msg->data));
        }
    );

    // Init freenect
    if (freenect_init(&f_ctx_, NULL) < 0) {
        RCLCPP_FATAL(get_logger(), "freenect_init() failed");
        rclcpp::shutdown();
    }
    freenect_set_log_level(f_ctx_, FREENECT_LOG_INFO);
    freenect_select_subdevices(f_ctx_, (freenect_device_flags)(FREENECT_DEVICE_CAMERA | FREENECT_DEVICE_MOTOR));

    int nr_devices = freenect_num_devices(f_ctx_);
    if (nr_devices > 1) {
        RCLCPP_INFO(get_logger(), "Found %d kinect devices, using the first one", nr_devices);
    } else if (nr_devices == 0) {
        RCLCPP_FATAL(get_logger(), "No kinect found");
        rclcpp::shutdown();
    }

    if (freenect_open_device(f_ctx_, &dev_, 0) < 0) {
        RCLCPP_FATAL(get_logger(), "Could not open Kinect");
        rclcpp::shutdown();
    }

    freenect_set_user(dev_, this);

    // Set callbacks
    freenect_set_video_callback(dev_, &KinectNode::video_cb);
    freenect_set_depth_callback(dev_, &KinectNode::depth_cb);

    // Set modes
    freenect_frame_mode rgb_mode = freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB);
    freenect_frame_mode depth_mode = freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_MM);
    freenect_set_video_mode(dev_, rgb_mode);
    freenect_set_depth_mode(dev_, depth_mode);

    // Enable automatic exposure
    freenect_set_flag(dev_, FREENECT_AUTO_EXPOSURE, FREENECT_ON);
    freenect_set_flag(dev_, FREENECT_AUTO_FLICKER, FREENECT_ON);
    freenect_set_flag(dev_, FREENECT_AUTO_WHITE_BALANCE, FREENECT_ON);

    // Start streams
    freenect_start_video(dev_);
    freenect_start_depth(dev_);

    // Processing thread
    kinect_thread_ = std::thread([this]() {
        while (rclcpp::ok()) {
            if (freenect_process_events(f_ctx_) < 0) break;
        }
        RCLCPP_FATAL(get_logger(), "Kinect error");
        rclcpp::shutdown();
    });
}

void KinectNode::timer_callback() {
    // Calculate tilt state
    freenect_update_tilt_state(this->dev_);

    // Extract
    auto tilt_state = freenect_get_tilt_state(this->dev_);
    auto tilt = freenect_get_tilt_degs(tilt_state);

    // Send
    std_msgs::msg::Float64 msg;
    msg.data = tilt;
    this->tilt_pub_->publish(msg);
}

KinectNode::~KinectNode()
{
    freenect_stop_video(dev_);
    freenect_stop_depth(dev_);
    freenect_close_device(dev_);
    freenect_shutdown(f_ctx_);
    if (kinect_thread_.joinable()) kinect_thread_.join();
}

void KinectNode::video_cb(freenect_device *dev, void *video, uint32_t /*timestamp*/)
{
    auto *node = static_cast<KinectNode*>(freenect_get_user(dev));
    sensor_msgs::msg::Image msg;
    auto stamp = node->now();
    msg.header.stamp = stamp;
    msg.header.frame_id = node->get_parameter("rgb_frame").as_string();
    msg.height = 480;
    msg.width = 640;
    msg.encoding = sensor_msgs::image_encodings::RGB8;
    msg.step = msg.width * 3;
    msg.data.resize(msg.step * msg.height);
    std::memcpy(msg.data.data(), video, msg.data.size());
    node->rgb_info_.header.stamp = stamp;
    node->rgb_pub_.publish(msg, node->rgb_info_);
}

void KinectNode::depth_cb(freenect_device *dev, void *depth, uint32_t /*timestamp*/)
{
    auto *node = static_cast<KinectNode*>(freenect_get_user(dev));
    sensor_msgs::msg::Image msg;
    auto stamp = node->now();
    msg.header.stamp = stamp;
    msg.header.frame_id = node->get_parameter("depth_frame").as_string();
    msg.height = 480;
    msg.width = 640;
    msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    msg.step = msg.width * 2;
    msg.data.resize(msg.step * msg.height);
    std::memcpy(msg.data.data(), depth, msg.data.size());
    node->depth_info_.header.stamp = stamp;
    node->depth_pub_.publish(msg, node->depth_info_);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(KinectNode)