/***********************************************************************
  Copyright (C) 2020 Hironori Fujimoto

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
***********************************************************************/

#include "sgm_gpu_node.h"
#include <cv_bridge/cv_bridge.h>
namespace sgm_gpu {

SgmGpuNode::SgmGpuNode() {
  node_handle_.reset(new ros::NodeHandle());
  private_node_handle_.reset(new ros::NodeHandle("~"));

  image_transport_.reset(new image_transport::ImageTransport(*node_handle_));

  sgm_.reset(new SgmGpu(*private_node_handle_));

  disparity_pub_ = private_node_handle_->advertise<stereo_msgs::DisparityImage>(
      "disparity", 1);

  // Subscribe left and right Image topic
  std::string left_base_topic = node_handle_->resolveName("left_image");
  std::string right_base_topic = node_handle_->resolveName("right_image");
  left_image_sub_.subscribe(*image_transport_, left_base_topic, 10);
  right_image_sub_.subscribe(*image_transport_, right_base_topic, 10);

  // Find CameraInfo topic from corresponded Image topic and subscribe it
  // std::string left_info_topic =
  // image_transport::getCameraInfoTopic(left_base_topic); std::string
  // right_info_topic = image_transport::getCameraInfoTopic(right_base_topic);

  std::string left_info_topic =
      "/airsim_node/drone_1/front_left/Scene/camera_info";
  std::string right_info_topic =
      "/airsim_node/drone_1/front_right/Scene/camera_info";

  left_info_sub_.subscribe(*node_handle_, left_info_topic, 10);
  right_info_sub_.subscribe(*node_handle_, right_info_topic, 10);

  // stereo_synchronizer_.reset(
  //   new StereoSynchronizer(left_image_sub_, right_image_sub_, left_info_sub_,
  //   right_info_sub_, 10)
  // );

  stereo_synchronizer_.reset(new message_filters::Synchronizer<SyncPolicy>(
      SyncPolicy(10), left_image_sub_, right_image_sub_, left_info_sub_,
      right_info_sub_));
  stereo_synchronizer_->registerCallback(&SgmGpuNode::stereoCallback, this);
}

void SgmGpuNode::stereoCallback(
    const sensor_msgs::ImageConstPtr &left_image,
    const sensor_msgs::ImageConstPtr &right_image,
    const sensor_msgs::CameraInfoConstPtr &left_info,
    const sensor_msgs::CameraInfoConstPtr &right_info) {
  // std::cout << "working!" << std::endl;
  // if (disparity_pub_.getNumSubscribers() == 0)
  //   return;

  stereo_msgs::DisparityImage disparity;
  sgm_->computeDisparity(*left_image, *right_image, *left_info, *right_info,
                         disparity);

  disparity_pub_.publish(disparity);

  // gs
  const sensor_msgs::Image &image_msg = disparity.image;

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(image_msg,
                                 sensor_msgs::image_encodings::TYPE_32FC1);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    std::cout << "not working!~" << std::endl;
    return; // Handle the conversion error
  }
  cv::Mat disparity_mat = cv_ptr->image;
  // turn disparity_mat to depth
  cv::Mat depth_mat =
      cv::Mat::zeros(disparity_mat.rows, disparity_mat.cols, CV_32FC1);
  for (int i = 0; i < disparity_mat.rows; i++) {
    for (int j = 0; j < disparity_mat.cols; j++) {
      float disparity = disparity_mat.at<float>(i, j);
      if (disparity > 0) {
        depth_mat.at<float>(i, j) = 1.0 / disparity;
      }
    }
  }
  cv::imshow("depth", depth_mat);
  cv::waitKey(1);
}

} // namespace sgm_gpu
