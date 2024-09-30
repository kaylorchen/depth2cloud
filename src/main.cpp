#include "kaylordut/log/logger.h"
#include "rclcpp/wait_for_message.hpp"
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Core>
#include "eigen3/Eigen/Dense"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include "depth2cloud.h"
#include <pcl_conversions/pcl_conversions.h>

class Depth2CloudNode : public rclcpp::Node {
 public:
  Depth2CloudNode() : Node("depth2cloud_node") {
  }

  void Init() {
    sensor_msgs::msg::CameraInfo camera_info;
    auto ptr = shared_from_this();
    if (rclcpp::wait_for_message<sensor_msgs::msg::CameraInfo>(camera_info,
                                                               ptr,
                                                               camera_info_topic_,
                                                               std::chrono::seconds(10))) {
      cam_param.width = camera_info.width;
      cam_param.height = camera_info.height;
      cam_param.binning_x = camera_info.binning_x;
      cam_param.binning_y = camera_info.binning_y;
      memcpy(cam_param.d.data(), camera_info.d.data(), sizeof(camera_info.d.at(0)) * camera_info.d.size());
      memcpy(cam_param.k.data(), camera_info.k.data(), sizeof(camera_info.k.at(0)) * camera_info.k.size());
      cam_param.k = cam_param.k.transpose().eval();

      point_cloud_publisher1_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud1", 1);
      point_cloud_publisher2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud2", 1);
      depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(depth_topic_,
                                                                      rclcpp::QoS(rclcpp::KeepLast(1)),
                                                                      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
                                                                        this->DepthImageCallback(msg);
                                                                      });

    }
    std::stringstream ss;
    ss << "Camera Info: " << std::endl;
    ss << "Width: " << cam_param.width << std::endl;
    ss << "Height: " << cam_param.height << std::endl;
    ss << "Distortion Coefficients: " << std::endl;
    ss << "d: " << std::endl;
    ss << cam_param.d << std::endl;
    ss << "K: " << std::endl;
    ss << cam_param.k << std::endl;
    ss << "Binning X: " << cam_param.binning_x << std::endl;
    ss << "Binning Y: " << cam_param.binning_y << std::endl;
    KAYLORDUT_LOG_INFO("{}", ss.str());
  }

 private:
  void DepthImageCallback(sensor_msgs::msg::Image::ConstSharedPtr msg) {
    KAYLORDUT_LOG_DEBUG("Received a depth image message");
    cv_bridge::CvImagePtr cv_image_ptr_;
    try {
      cv_image_ptr_ = cv_bridge::toCvCopy(msg, msg->encoding);
      depth_image_ = cv_image_ptr_->image;
    } catch (const cv::Exception &e) {
      KAYLORDUT_LOG_ERROR("cv_bridge error: {}", e.what());
      return;
    }
    Depth2Cloud::Box box;
    box.x2 = depth_image_.cols;
    box.y2 = depth_image_.rows;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    KAYLORDUT_TIME_COST_INFO("convert depth image to cloud1",
                             cloud = depth2cloud_.Convert(depth_image_, cam_param, box)
    );
    KAYLORDUT_LOG_INFO("cloud1 size: {}", cloud->size());
    sensor_msgs::msg::PointCloud2 msg_cloud;
    pcl::toROSMsg(*cloud, msg_cloud);
    msg_cloud.header = msg->header;
//    msg_cloud.header.stamp = this->now();
    point_cloud_publisher1_->publish(msg_cloud);
//    box.x2 = depth_image_.cols * 0.5;
//    box.y2 = depth_image_.rows * 0.5;
    KAYLORDUT_TIME_COST_INFO("convert depth image to cloud2",
                             cloud = depth2cloud_.Convert(depth_image_, cam_param, box, 1000.0, 0.5, 0.5)
    );
    KAYLORDUT_LOG_INFO("cloud2 size: {}", cloud->size());
    pcl::toROSMsg(*cloud, msg_cloud);
    msg_cloud.header = msg->header;
    point_cloud_publisher2_->publish(msg_cloud);
  }


  std::string camera_info_topic_{"/camera_01/depth/camera_info"};
  Depth2Cloud::CameraInfo cam_param;
  std::string depth_topic_{"/camera_01/depth/image_raw"};
  rclcpp::SubscriptionBase::SharedPtr depth_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher1_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher2_;
  cv::Mat depth_image_;
  Depth2Cloud depth2cloud_;
};

int main(int argc, char **argv) {
  KAYLORDUT_LOG_INFO("depth2cloud demo");
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Depth2CloudNode>();
  node->Init();
  rclcpp::spin(node);
  return 0;
}
