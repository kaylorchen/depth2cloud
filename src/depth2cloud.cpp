//
// Created by kaylor on 9/29/24.
//

#include "depth2cloud.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr Depth2Cloud::Convert(const cv::Mat &input,
                                                         const Depth2Cloud::CameraInfo &camera_info,
                                                         const Depth2Cloud::Box &box,
                                                         float depth_scale,
                                                         float scale_x,
                                                         float scale_y) {
  int resized_width = input.cols * scale_x;
  int resized_height = input.rows * scale_y;
  auto fx = camera_info.k(0, 0) * scale_x;
  auto fy = camera_info.k(1, 1) * scale_y;
  auto cx = camera_info.k(0, 2) * scale_x;
  auto cy = camera_info.k(1, 2) * scale_y;
  cv::Mat depth_image;
  Box vaild_box = {0};
  if (resized_height == input.rows && resized_width == input.cols){
    depth_image = input;
    vaild_box = box;
  }else{
    cv::resize(input, depth_image, cv::Size(resized_width, resized_height), 0, 0, cv::INTER_NEAREST);
    vaild_box.x1 = box.x1 * scale_x;
    vaild_box.y1 = box.y1 * scale_y;
    vaild_box.x2 = box.x2 * scale_x;
    vaild_box.y2 = box.y2 * scale_y;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  float depth = 0;
  int row_num = std::abs(vaild_box.y2 - vaild_box.y1);
  int col_num = std::abs(vaild_box.x2 - vaild_box.x1);
  (cloud->points).reserve(row_num * col_num);
  for (int row = vaild_box.y1; row < vaild_box.y2; row++) {
    for (int col = vaild_box.x1; col < vaild_box.x2; col++) {
      depth = depth_image.at<uint16_t>(row, col) / depth_scale;
      pcl::PointXYZ point(0.0, 0.0, 0.0);
      if ((depth > 0.000) && (depth < 3.000)) {
        point.z = depth;
        point.x = (col - cx) * point.z / fx;
        point.y = (row - cy) * point.z / fy;
      }
      cloud->emplace_back(std::move(point));
    }
  }
  return cloud;
}