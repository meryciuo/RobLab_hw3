
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.
 
using namespace std::chrono_literals;

using std::placeholders::_1;
using namespace cv;
 
class MinimalImagePublisher : public rclcpp::Node {
public:
  MinimalImagePublisher() : Node("opencv_image_publisher"), count_(0) {

    subscriber_ = 
        this->create_subscription<sensor_msgs::msg::Image>
        ("/videocamera", 10, std::bind(&MinimalImagePublisher::imageCallback, this,_1));

    publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("/videocamera_cv", 10);

        RCLCPP_INFO(this->get_logger(), "Node initialized");

    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalImagePublisher::timer_callback, this));
  }
 
private:
  
  Mat processed_image_;
  std_msgs::msg::Header processed_image_header;
 
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg){
    
    Mat current_image = cv_bridge::toCvCopy(msg,"bgr8")->image;
    cv::imshow("Image",current_image);
    cv::waitKey(1);
    RCLCPP_INFO(this->get_logger(), "Received image");

    SimpleBlobDetector::Params params;
     // Change thresholds
    params.minThreshold = 50;
    params.maxThreshold = 200;
    
    // Filter by Area.
    params.filterByArea = false;
    params.minArea = 1500;
    
    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0.8;
    
    // Filter by Convexity
    params.filterByConvexity = false;
    params.minConvexity = 0.9;
    
    // Filter by Inertia
    params.filterByInertia = false;
    params.minInertiaRatio = 0.01;

    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
    
    std::vector<KeyPoint> keypoints;
    detector->detect(current_image, keypoints);

    
    drawKeypoints(current_image ,keypoints, processed_image_, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

    processed_image_header=msg->header;
 
    // Show blobs
    imshow("keypoints", processed_image_ );
    waitKey(1);

  }

  void timer_callback() {
 
     // Write message to be sent. Member function toImageMsg() converts a CvImage
    // into a ROS image message
    msg_ = cv_bridge::CvImage(processed_image_header, "bgr8", processed_image_)
               .toImageMsg();
    // Publish the image to the topic defined in the publisher
    publisher_->publish(*msg_.get());
    RCLCPP_INFO(this->get_logger(), "Processed image published,",count_);
    count_++;
  }

  
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Image::SharedPtr msg_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
  


  size_t count_;
};
 
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // create a ros2 node
  auto node = std::make_shared<MinimalImagePublisher>();
 
  // process ros2 callbacks until receiving a SIGINT (ctrl-c)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

