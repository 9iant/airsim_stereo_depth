// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>

#include <opencv2/highgui.hpp>

#include "common/common_utils/StrictMode.hpp"
#include "yaml-cpp/yaml.h"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif  // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include <chrono>
#include <iostream>

#include "airsim_stereo/airsim_settings_parser.h"  // airsim settings parser
#include "common/AirSimSettings.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "sgm_gpu/sgm_gpu.h"  // stereo matching
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

#define M_PI 3.14159265358979323846

// sensor_msgs::CameraInfo
// generate_cam_info(const std::string &camera_name,
//                   const msr::airlib::CaptureSetting &capture_setting) const {
//   sensor_msgs::CameraInfo cam_info_msg;
//   cam_info_msg.header.frame_id = camera_name;
//   cam_info_msg.distortion_model = "plumb_bob";
//   cam_info_msg.height = capture_setting.height;
//   cam_info_msg.width = capture_setting.width;
// float f_x = (capture_setting.width / 2.0) /
//             tan(math_common::deg2rad(capture_setting.fov_degrees / 2.0));
//   // todo focal length in Y direction should be same as X it seems. this
//   can change in future a scene capture component which exactly correponds to
//   a
//       cine camera
//           // float f_y = (capture_setting.height / 2.0) /
//           tan(math_common::deg2rad(fov_degrees / 2.0));
//   cam_info_msg.K = {f_x, 0.0, capture_setting.width / 2.0,
//                     0.0, f_x, capture_setting.height / 2.0,
//                     0.0, 0.0, 1.0};
//   cam_info_msg.P = {f_x,
//                     0.0,
//                     capture_setting.width / 2.0,
//                     0.0,
//                     0.0,
//                     f_x,
//                     capture_setting.height / 2.0,
//                     0.0,
//                     0.0,
//                     0.0,
//                     1.0,
//                     0.0};
//   return cam_info_msg;
// }

int main(int argc, char** argv) {
  using namespace msr::airlib;
  // connect to the AirSim simulator
  msr::airlib::MultirotorRpcLibClient client;
  typedef ImageCaptureBase::ImageRequest ImageRequest;
  typedef ImageCaptureBase::ImageResponse ImageResponse;
  typedef ImageCaptureBase::ImageType ImageType;
  typedef common_utils::FileSystem FileSystem;
  typedef msr::airlib::AirSimSettings AirSimSettings;
  typedef msr::airlib::SensorBase SensorBase;
  typedef msr::airlib::AirSimSettings::CameraSetting CameraSetting;
  typedef msr::airlib::AirSimSettings::CaptureSetting CaptureSetting;

  ros::init(argc, argv, "airsim_image_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  std::string host_ip = "localhost";
  nh_private.getParam("host_ip", host_ip);

  std::cout << "working1" << std::endl;

  //   const AirSimSettings::VehicleSetting* vehicle_setting =
  //       AirSimSettings::singleton().getVehicleSetting("Drone1");
  //   for (const auto& curr_vehicle_elem :
  //   AirSimSettings::singleton().vehicles) {
  //     auto& vehicle_setting = curr_vehicle_elem.second;
  //     auto curr_vehicle_name = curr_vehicle_elem.first;

  //     nh.setParam("/vehicle_name", curr_vehicle_name);
  //     std::cout << curr_vehicle_name << std::endl;
  //     std::cout << "vehicle_name" << std::endl;
  //   }
  std::cout << "working2" << std::endl;

  // stereo matching
  int img_cols_;
  int img_rows_;
  double rgb_fov_deg_;
  double stereo_baseline_;

  nh_private.param("img_cols", img_cols_, 320);
  nh_private.param("img_rows", img_rows_, 240);
  nh_private.param("rgb_fov_deg", rgb_fov_deg_, 90.0);
  nh_private.param("stereo_baseline", stereo_baseline_, 0.1);

  std::shared_ptr<sgm_gpu::SgmGpu> sgm_;
  sgm_.reset(new sgm_gpu::SgmGpu(nh_private, img_cols_, img_rows_));

  image_transport::ImageTransport it(nh);

  // create image transport publishers for left and right images
  //   image_transport::Publisher left_pub = it.advertise("left_image", 1);
  //   image_transport::Publisher right_pub = it.advertise("right_image", 1);
  image_transport::Publisher depth_pub = it.advertise("depth_image", 10);

  ros::Rate loop_rate(50);  // we will target 30 fps

  client.confirmConnection();

  std::cout << "connected!" << std::endl;

  const std::vector<ImageRequest> request{
      ImageRequest("front_left", ImageType::Scene),
      ImageRequest("front_right", ImageType::Scene)};

  while (ros::ok()) {
    // start time
    std::chrono::steady_clock::time_point begin_time =
        std::chrono::steady_clock::now();

    const std::vector<ImageResponse>& response = client.simGetImages(request);

    std::chrono::steady_clock::time_point after_get_image =
        std::chrono::steady_clock::now();

    const ImageResponse& left_image = response.at(0);
    const ImageResponse& right_image = response.at(1);

    cv::Mat left_mat =
        cv::imdecode(left_image.image_data_uint8, cv::IMREAD_COLOR);
    cv::Mat right_mat =
        cv::imdecode(right_image.image_data_uint8, cv::IMREAD_COLOR);
    // cv::imshow("left", left_mat);
    // cv::imshow("right", right_mat);
    // cv::waitKey(1);

    // compute depth
    // ros::WallTime start_disp_comp = ros::WallTime::now();
    cv::Mat disparity(img_rows_, img_cols_, CV_8UC1);
    sgm_->computeDisparity(left_mat, right_mat, &disparity);
    disparity.convertTo(disparity, CV_32FC1);

    // compute depth from disparity
    cv::Mat depth_float(img_rows_, img_cols_, CV_32FC1);
    cv::Mat depth_uint16(img_rows_, img_cols_, CV_16UC1);

    float f =
        (img_cols_ / 2.0) / std::tan((M_PI * (rgb_fov_deg_ / 180.0)) / 2.0);
    //  depth = static_cast<float>(stereo_baseline_) * f / disparity;
    for (int r = 0; r < img_rows_; ++r) {
      for (int c = 0; c < img_cols_; ++c) {
        if (disparity.at<float>(r, c) == 0.0f) {
          depth_float.at<float>(r, c) = 0.0f;
          depth_uint16.at<unsigned short>(r, c) = 0;
        } else if (disparity.at<float>(r, c) == 255.0f) {
          depth_float.at<float>(r, c) = 0.0f;
          depth_uint16.at<unsigned short>(r, c) = 0;
        } else {
          depth_float.at<float>(r, c) = static_cast<float>(stereo_baseline_) *
                                        f / disparity.at<float>(r, c);
          depth_uint16.at<unsigned short>(r, c) = static_cast<unsigned short>(
              1000.0 * static_cast<float>(stereo_baseline_) * f /
              disparity.at<float>(r, c));
        }
      }
    }
    // double disp_comp_duration =
    //     (ros::WallTime::now() - start_disp_comp).toSec();

    sensor_msgs::ImagePtr depth_msg =
        cv_bridge::CvImage(std_msgs::Header(), "mono16", depth_uint16)
            .toImageMsg();
    depth_pub.publish(depth_msg);

    std::chrono::steady_clock::time_point after_pub_image =
        std::chrono::steady_clock::now();

    // print times
    std::cout << "get image: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(
                     after_get_image - begin_time)
                     .count()
              << " ms" << std::endl;
    std::cout << "compute depth and pub : "
              << std::chrono::duration_cast<std::chrono::milliseconds>(
                     after_pub_image - after_get_image)
                     .count()
              << " ms" << std::endl;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}