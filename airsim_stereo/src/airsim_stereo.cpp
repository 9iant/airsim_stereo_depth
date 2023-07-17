// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
#include "common/common_utils/StrictMode.hpp"
#include "yaml-cpp/yaml.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "common/AirSimSettings.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "sgm_gpu/sgm_gpu.h" // stereo matching
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <chrono>
#include <iostream>

static constexpr char CAM_YML_NAME[] = "camera_name";
static constexpr char WIDTH_YML_NAME[] = "image_width";
static constexpr char HEIGHT_YML_NAME[] = "image_height";
static constexpr char K_YML_NAME[] = "camera_matrix";
static constexpr char D_YML_NAME[] = "distortion_coefficients";
static constexpr char R_YML_NAME[] = "rectification_matrix";
static constexpr char P_YML_NAME[] = "projection_matrix";
static constexpr char DMODEL_YML_NAME[] = "distortion_model";

// define mathmatical parameters s.t. pi, degree to radian, etc.
#define M_PI 3.14159265358979323846

// void computDepthImage(const cv::Mat& left_frame, const cv::Mat& right_frame,
// cv::Mat* const depth){

// }

// sensor_msgs::CameraInfo generate_cam_info(const std::string& camera_name,
//                                                             const
//                                                             CameraSetting&
//                                                             camera_setting,
//                                                             const
//                                                             CaptureSetting&
//                                                             capture_setting)
//                                                             const
// {
//     sensor_msgs::CameraInfo cam_info_msg;
//     cam_info_msg.header.frame_id = camera_name;
//     cam_info_msg.distortion_model = "plumb_bob";
//     cam_info_msg.height = capture_setting.height;
//     cam_info_msg.width = capture_setting.width;
//     float f_x = (capture_setting.width / 2.0) /
//     tan(math_common::deg2rad(capture_setting.fov_degrees / 2.0));
//     // todo focal length in Y direction should be same as X it seems. this
//     can change in future a scene capture component which exactly correponds
//     to a cine camera
//     // float f_y = (capture_setting.height / 2.0) /
//     tan(math_common::deg2rad(fov_degrees / 2.0)); cam_info_msg.K = { f_x,
//     0.0, capture_setting.width / 2.0, 0.0, f_x, capture_setting.height / 2.0,
//     0.0, 0.0, 1.0 }; cam_info_msg.P = { f_x, 0.0, capture_setting.width
//     / 2.0, 0.0, 0.0, f_x, capture_setting.height / 2.0, 0.0, 0.0, 0.0, 1.0,
//     0.0 }; return cam_info_msg;
// }
// void read_params_from_yaml_and_fill_cam_info_msg(
//     const std::string &file_name, sensor_msgs::CameraInfo &cam_info) const {
//   std::ifstream fin(file_name.c_str());
//   YAML::Node doc = YAML::Load(fin);

//   cam_info.width = doc[WIDTH_YML_NAME].as<int>();
//   cam_info.height = doc[HEIGHT_YML_NAME].as<int>();

//   SimpleMatrix K_(3, 3, &cam_info.K[0]);
//   convert_yaml_to_simple_mat(doc[K_YML_NAME], K_);
//   SimpleMatrix R_(3, 3, &cam_info.R[0]);
//   convert_yaml_to_simple_mat(doc[R_YML_NAME], R_);
//   SimpleMatrix P_(3, 4, &cam_info.P[0]);
//   convert_yaml_to_simple_mat(doc[P_YML_NAME], P_);

//   cam_info.distortion_model = doc[DMODEL_YML_NAME].as<std::string>();

//   const YAML::Node &D_node = doc[D_YML_NAME];
//   int D_rows, D_cols;
//   D_rows = D_node["rows"].as<int>();
//   D_cols = D_node["cols"].as<int>();
//   const YAML::Node &D_data = D_node["data"];
//   cam_info.D.resize(D_rows * D_cols);
//   for (int i = 0; i < D_rows * D_cols; ++i) {
//     cam_info.D[i] = D_data[i].as<float>();
//   }
// }

int main(int argc, char **argv) {

  using namespace msr::airlib;
  // connect to the AirSim simulator
  msr::airlib::MultirotorRpcLibClient client;
  typedef ImageCaptureBase::ImageRequest ImageRequest;
  typedef ImageCaptureBase::ImageResponse ImageResponse;
  typedef ImageCaptureBase::ImageType ImageType;
  typedef common_utils::FileSystem FileSystem;

  ros::init(argc, argv, "airsim_image_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  std::string host_ip = "localhost";
  nh_private.getParam("host_ip", host_ip);

  // stereo matching
  int img_cols_ = 540;
  int img_rows_ = 360;
  float rgb_fov_deg_ = 90.0;
  float stereo_baseline_ = 0.1;
  std::shared_ptr<sgm_gpu::SgmGpu> sgm_;
  sgm_.reset(new sgm_gpu::SgmGpu(nh_private, img_cols_, img_rows_));

  image_transport::ImageTransport it(nh);

  // create image transport publishers for left and right images
  image_transport::Publisher left_pub = it.advertise("left_image", 1);
  image_transport::Publisher right_pub = it.advertise("right_image", 1);

  ros::Rate loop_rate(30); // we will target 30 fps

  client.confirmConnection();

  std::cout << "connected!" << std::endl;

  const std::vector<ImageRequest> request{
      ImageRequest("front_left", ImageType::Scene),
      ImageRequest("front_right", ImageType::Scene)};
  //    response = client.simGetImages(request);

  //   std::cout << "# of images received: " << response.size() << std::endl;

  while (ros::ok()) {

    const std::vector<ImageResponse> &response = client.simGetImages(request);

    const ImageResponse &left_image = response.at(0);
    const ImageResponse &right_image = response.at(1);

    cv::Mat left_mat =
        cv::imdecode(left_image.image_data_uint8, cv::IMREAD_COLOR);
    cv::Mat right_mat =
        cv::imdecode(right_image.image_data_uint8, cv::IMREAD_COLOR);
    cv::imshow("left", left_mat);
    cv::imshow("right", right_mat);
    // cv::waitKey(1);

    // compute depth
    ros::WallTime start_disp_comp = ros::WallTime::now();
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
    double disp_comp_duration =
        (ros::WallTime::now() - start_disp_comp).toSec();

    cv::imshow("depth", depth_uint16);
    cv::waitKey(1);

    // for (const ImageResponse &image_info : response) {
    //   std::cout << "working!" << std::endl;
    //   std::cout << "Image uint8 size: " << image_info.image_data_uint8.size()
    //             << std::endl;

    //   cv::Mat img = cv::imdecode(image_info.image_data_uint8,
    //   cv::IMREAD_COLOR);
    // }
    // if (response.size()) {

    //   for (const ImageResponse &image_info : response) {

    //     if (path != "") {
    //       std::string file_path =
    //           FileSystem::combine(path,
    //           std::to_string(image_info.time_stamp));
    //       if (image_info.pixels_as_float) {
    //         Utils::writePFMfile(image_info.image_data_float.data(),
    //                             image_info.width, image_info.height,
    //                             file_path + ".pfm");
    //       } else {
    //         std::ofstream file(file_path + ".png", std::ios::binary);
    //         file.write(reinterpret_cast<const char *>(
    //                        image_info.image_data_uint8.data()),
    //                    image_info.image_data_uint8.size());
    //         file.close();
    //       }
    //     }
    //   }
    // }
    // // get left and right images from AirSim
    // const auto &left_response =
    //     client.simGetImage("front_left", ImageType::Scene);
    // // auto right_image = client.simGetImage("fribt_right",
    // ImageType::Scene);

    // if (left_image.size() > 1) {
    //   // convert the images to cv::Mat
    //   cv::Mat left_mat =
    //       cv::imdecode(left_response.image_data_uint8, cv::IMREAD_COLOR);
    //   //   cv::Mat right_mat = cv::imdecode(right_image,
    //   cv::IMREAD_COLOR);

    //   // convert the cv::Mat images to ROS image messages
    //   sensor_msgs::ImagePtr left_msg =
    //       cv_bridge::CvImage(std_msgs::Header(), "bgr8",
    //       left_mat).toImageMsg();

    //   // publish the images
    //   left_pub.publish(left_msg);
    // }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}