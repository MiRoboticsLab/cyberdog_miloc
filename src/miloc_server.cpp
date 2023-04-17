// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <chrono>
#include <vector>
#include <cmath>
#include <memory>
#include <utility>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "cv_bridge/cv_bridge.h"
#include "tf2/utils.h"
#include "cyberdog_common/cyberdog_log.hpp"

#include "cyberdog_miloc/miloc_server.hpp"

namespace cyberdog
{
namespace miloc
{

using namespace std::chrono_literals;

int MilocServer::ShutDown()
{
  int ret = miloc_api_->ShutDown();
  miloc_api_.reset();

  return ret;
}

int MilocServer::Init()
{
  miloc_api_ = std::make_shared<MilocApi>();

  std::string config_path(param<std::string>(
      this, "config_path",
      "/SDCARD/miloc/config/config.yml"));

  int ret = miloc_api_->StartUp(config_path);

  current_map_id = -1;

  miloc_status_timer_ =
    this->create_wall_timer(1s, std::bind(&MilocServer::MilocStatusCallback, this));
  miloc_status_publisher_ = this->create_publisher<cyberdog_visions_interfaces::msg::MilocStatus>(
    "miloc_status", 10);
  reloc_result_publisher_ = this->create_publisher<std_msgs::msg::Int32>("reloc_result", 10);

  create_map_client_ = this->create_client<std_srvs::srv::SetBool>(
    "cyberdog_occmap/create_map_service");
  finish_map_client_ = this->create_client<cyberdog_visions_interfaces::srv::FinishMap>(
    "cyberdog_occmap/finish_map_service");

  reloc_service_ = this->create_service<cyberdog_visions_interfaces::srv::Reloc>(
    "reloc_service",
    std::bind(
      &MilocServer::RelocCallback, this, std::placeholders::_1,
      std::placeholders::_2));

  create_map_service_ = this->create_service<std_srvs::srv::SetBool>(
    "create_map_service",
    std::bind(
      &MilocServer::CreateMapCallback, this, std::placeholders::_1,
      std::placeholders::_2));
  finish_map_service_ = this->create_service<cyberdog_visions_interfaces::srv::FinishMap>(
    "finish_map_service",
    std::bind(
      &MilocServer::FinishMapCallback, this, std::placeholders::_1,
      std::placeholders::_2));
  start_nav_service_ = this->create_service<std_srvs::srv::SetBool>(
    "start_nav_service",
    std::bind(
      &MilocServer::StartNavigationCallback, this, std::placeholders::_1,
      std::placeholders::_2));
  stop_nav_service_ = this->create_service<std_srvs::srv::SetBool>(
    "stop_nav_service",
    std::bind(
      &MilocServer::StopNavigationCallback, this, std::placeholders::_1,
      std::placeholders::_2));

  get_miloc_status_ = this->create_service<cyberdog_visions_interfaces::srv::MilocMapHandler>(
    "get_miloc_status",
    std::bind(
      &MilocServer::GetMapStatusCallback, this, std::placeholders::_1,
      std::placeholders::_2));

  delete_map_service_ = this->create_service<cyberdog_visions_interfaces::srv::MilocMapHandler>(
    "delete_reloc_map",
    std::bind(
      &MilocServer::DeleteMapCallback, this, std::placeholders::_1,
      std::placeholders::_2));

  collect_image_thread_ = std::unique_ptr<std::thread>(
    new std::thread(&MilocServer::CollectImage, this));

  connector_sub_ = create_subscription<protocol::msg::ConnectorStatus>(
    "connector_state", rclcpp::SystemDefaultsQoS(),
    std::bind(&MilocServer::ModelDownloadCallback, this, std::placeholders::_1));

  miloc_model_ = std::make_shared<cyberdog::common::cyberdog_model>(
    "models", false, "2.0",
    "/SDCARD/", "miloc");

  reloc_failure_threshold_ = param<int>(this, "reloc_failure_threshold", 20);

  immediately_reconstruct_ = param<bool>(this, "immediately_reconstruct", true);

  return ret;
}

int MilocServer::Subscribe()
{
  // Subscribe /odom_slam
  std::string odom_topic(param<std::string>(this, "odom_slam", "/odom_slam"));
  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic,
    10,
    std::bind(&MilocServer::OdometryCallback, this, std::placeholders::_1));

  // Subscribe /odom_out
  std::string odom_leg_topic(param<std::string>(this, "odom_out", "/odom_out"));
  odom_leg_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_leg_topic,
    10,
    std::bind(&MilocServer::OdometryLegCallback, this, std::placeholders::_1));

  // Subscribe triple camera
  std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr,
    const sensor_msgs::msg::Image::ConstSharedPtr,
    const sensor_msgs::msg::Image::ConstSharedPtr)>
  cb = std::bind(
    &MilocServer::TripleCameraCallback, this, std::placeholders::_1,
    std::placeholders::_2, std::placeholders::_3);

  std::string cam0_topic(param<std::string>(this, "cam0_topic", "/image_rgb"));
  std::string cam1_topic(param<std::string>(this, "cam1_topic", "/image_left"));
  std::string cam2_topic(param<std::string>(this, "cam2_topic", "/image_right"));

  auto sub_image0 = std::make_unique<ImageSyncSub>(
    this, cam0_topic);
  auto sub_image1 = std::make_unique<ImageSyncSub>(
    this, cam1_topic);
  auto sub_image2 = std::make_unique<ImageSyncSub>(
    this, cam2_topic);

  auto image_sync_policy_triple = std::make_unique<ImageSyncPolicyTriple>(10);
  image_sync_policy_triple->setInterMessageLowerBound(0, rclcpp::Duration(0, 3000000));
  image_sync_policy_triple->setInterMessageLowerBound(1, rclcpp::Duration(0, 3000000));
  image_sync_policy_triple->setInterMessageLowerBound(2, rclcpp::Duration(0, 3000000));

  auto sync = std::make_unique<ImageSyncTriple>(
    (ImageSyncPolicyTriple)(*image_sync_policy_triple), *sub_image0, *sub_image1, *sub_image2);

  sync->registerCallback(
    std::bind(cb, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  sync_cam_triple_.push_back(std::move(sync));
  sync_subs_cam_.push_back(std::move(sub_image0));
  sync_subs_cam_.push_back(std::move(sub_image1));
  sync_subs_cam_.push_back(std::move(sub_image2));

  return 0;
}

int MilocServer::Unsubscribe()
{
  odom_subscriber_.reset();
  odom_leg_subscriber_.reset();
  sync_cam_triple_.clear();
  sync_subs_cam_.clear();

  while (!img_buffer_.empty()) {
    img_buffer_.pop_front();
  }
  while (!pose_buffer_.empty()) {
    pose_buffer_.pop_front();
  }

  return 0;
}

void MilocServer::ResetMiloc()
{
  int miloc_status = miloc_api_->GetMilocStatus();
  if (miloc_status == MilocStatus::kRELOCALIZABLE ||
    miloc_status == MilocStatus::kRELOCATING)
  {
    miloc_api_->StopNavigation();
    first_reloc_success_ = false;
    Unsubscribe();
  } else if (miloc_status == MilocStatus::kMAPPABLE ||
    miloc_status == MilocStatus::kMAPPING)
  {
    miloc_api_->FinishMap();
    Unsubscribe();
  }
}

int MilocServer::ModelCheck()
{
  //model_version less than 2.0 can not work

  std::string local_config_dir = "/SDCARD/miloc/models/" + std::string("version.toml");
  std::string local_model_version{"0.0"};

  if (access(local_config_dir.c_str(), F_OK) != 0) {
    local_model_version = "1.0";
  } else {
    bool ret = miloc_model_->Get_Model_Version(local_config_dir, local_model_version);
    if (!ret) {
      ERROR("get local model version fail...");
      return SLAM_ERROR;
    }
  }

  if (local_model_version.compare("2.0") < 0) {
    ERROR("local model version less than 2.0, can not work, please update");
    return SLAM_ERROR;
  }

  return SLAM_OK;
}

void MilocServer::ModelDownloadCallback(const protocol::msg::ConnectorStatus::SharedPtr msg)
{
  if (msg->is_internet && connector_sub_ != nullptr) {
    INFO("internet is ok, start to check miloc models");
    miloc_model_->SetTimeout(600);
    int32_t code = miloc_model_->UpdateModels();
    if (0 == code) {
      INFO("Update model from Fds success.");

      if (miloc_model_->Load_Model_Check()) {
        miloc_model_->Post_Process();
        INFO("replace and remove temp model prepare load new model");
      } else {
        INFO("load model without replace new model");
      }

    } else if (5829 == code) {
      INFO("model already downloads, do not need to update models");

      if (miloc_model_->Load_Model_Check()) {
        miloc_model_->Post_Process();
        INFO("replace and remove temp model prepare load new model");
      } else {
        INFO("load model without replace new model");
      }

    } else {
      ERROR("Update model from Fds fail.");
    }

    connector_sub_.reset();
    connector_sub_ = nullptr;
  }
}

void MilocServer::MilocStatusCallback()
{
  auto miloc_status_msg = cyberdog_visions_interfaces::msg::MilocStatus();
  miloc_status_msg.miloc_status = miloc_api_->GetMilocStatus();
  miloc_status_msg.current_map_id = miloc_api_->GetCurrentMapId();
  miloc_status_publisher_->publish(miloc_status_msg);
}

void MilocServer::TripleCameraCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr msg_front,
  const sensor_msgs::msg::Image::ConstSharedPtr msg_left,
  const sensor_msgs::msg::Image::ConstSharedPtr msg_right)
{
  cv::Mat img_front, img_left, img_right;

  img_front = cv_bridge::toCvShare(msg_front, "mono8")->image;
  img_left = cv_bridge::toCvShare(msg_left, "mono8")->image;
  img_right = cv_bridge::toCvShare(msg_right, "mono8")->image;

  ImgData img_data;
  img_front.copyTo(img_data.img_front);
  img_left.copyTo(img_data.img_left);
  img_right.copyTo(img_data.img_right);
  img_data.time_stamp = rclcpp::Time(msg_front->header.stamp).nanoseconds();

  img_lock_.lock();
  while (img_buffer_.size() > img_buffer_size_) {
    img_buffer_.pop_front();
  }
  img_buffer_.push_back(img_data);
  img_lock_.unlock();
}

void MilocServer::OdometryCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr msg_odom)
{
  pose_lock_.lock();
  while (pose_buffer_.size() > pose_buffer_size_) {
    pose_buffer_.pop_front();
  }
  geometry_msgs::msg::PoseStamped pose;
  pose.header = msg_odom->header;
  pose.pose = msg_odom->pose.pose;
  pose_buffer_.push_back(pose);
  pose_lock_.unlock();
}

void MilocServer::OdometryLegCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr msg_odom)
{
  odom_lock_.lock();
  while (odom_buffer_.size() > odom_buffer_size_) {
    odom_buffer_.pop_front();
  }
  nav_msgs::msg::Odometry odom;
  odom.header = msg_odom->header;
  odom.child_frame_id = msg_odom->child_frame_id;
  odom.pose = msg_odom->pose;
  odom.twist = msg_odom->twist;
  odom_buffer_.push_back(odom);
  odom_lock_.unlock();
}

void MilocServer::RelocCallback(
  const std::shared_ptr<cyberdog_visions_interfaces::srv::Reloc::Request> request,
  const std::shared_ptr<cyberdog_visions_interfaces::srv::Reloc::Response> response)
{
  if (miloc_api_->GetMilocStatus() != MilocStatus::kRELOCALIZABLE) {
    response->reply.status = cyberdog_visions_interfaces::msg::Reply::STATUS_ERROR;
    response->reply.status_msg = "miloc Status ERROR!";
    return;
  }

  INFO("start to reloc, reloc_id=%d", request->reloc_id);

  ImgData img_data;
  geometry_msgs::msg::PoseStamped pose_init;
  std_msgs::msg::Int32 miloc_status;

  img_lock_.lock();
  if (!img_buffer_.empty()) {
    img_data = img_buffer_.back();
    img_lock_.unlock();
    pose_lock_.lock();
    if (!pose_buffer_.empty()) {
      pose_init = pose_buffer_.back();
    }
    pose_lock_.unlock();

    std::vector<cv::Mat> image_list;
    image_list.push_back(img_data.img_front);
    image_list.push_back(img_data.img_left);
    image_list.push_back(img_data.img_right);

    Eigen::Quaterniond orientation;
    Eigen::Quaterniond orientation_init{0.0, 0.0, 0.0, 0.0};
    Eigen::Vector3d position;
    Eigen::Vector3d position_init{0.0, 0.0, 0.0};

    int64_t timestamp = img_data.time_stamp;
    int64_t timestamp_pose = rclcpp::Time(pose_init.header.stamp).nanoseconds();
    bool is_verified{false};

    if (timestamp_pose != 0 && std::abs(timestamp_pose - timestamp) <= 5e8) {
      orientation_init.x() = pose_init.pose.orientation.x;
      orientation_init.y() = pose_init.pose.orientation.y;
      orientation_init.z() = pose_init.pose.orientation.z;
      orientation_init.w() = pose_init.pose.orientation.w;

      position_init.x() = pose_init.pose.position.x;
      position_init.y() = pose_init.pose.position.y;
      position_init.z() = pose_init.pose.position.z;

      is_verified = true;
    }

    int reply_status{-1};
    float confidence{0.0};

    miloc_api_->EstimatePose(
      image_list, orientation, position, orientation_init, position_init,
      reply_status, confidence);

    response->reloc_id = request->reloc_id;
    response->is_verified = is_verified;
    response->pose.header.set__stamp(rclcpp::Time(img_data.time_stamp));
    response->confidence = confidence;
    if (cyberdog_visions_interfaces::msg::Reply::FIELD_ERROR == reply_status) {
      INFO("reloc fail");
      response->reply.status = cyberdog_visions_interfaces::msg::Reply::FIELD_ERROR;
      response->reply.status_msg = "reloc fail";

      if (0 == request->reloc_id && false == first_reloc_success_) {
        fail_num_ += 1;
        if (fail_num_ >= reloc_failure_threshold_) {
          INFO("first reloc fail, publish 200");
          miloc_status.data = 200;
          reloc_result_publisher_->publish(miloc_status);
          response->reply.status = 200;
        } else {
          INFO("first reloc fail, publish 100");
          miloc_status.data = 100;
          reloc_result_publisher_->publish(miloc_status);
        }
      }

    } else if (cyberdog_visions_interfaces::msg::Reply::SUCCESS == reply_status) {
      INFO("reloc success");
      response->reply.status = cyberdog_visions_interfaces::msg::Reply::SUCCESS;
      response->reply.status_msg = "reloc success";

      response->pose.pose.position.x = position.x();
      response->pose.pose.position.y = position.y();
      response->pose.pose.position.z = position.z();

      response->pose.pose.orientation.x = orientation.x();
      response->pose.pose.orientation.y = orientation.y();
      response->pose.pose.orientation.z = orientation.z();
      response->pose.pose.orientation.w = orientation.w();

      if (0 == request->reloc_id) {
        INFO("first reloc success, publish 0");
        fail_num_ = 0;
        miloc_status.data = 0;
        reloc_result_publisher_->publish(miloc_status);
        first_reloc_success_ = true;
      }

    } else if (cyberdog_visions_interfaces::msg::Reply::STATUS_ERROR == reply_status) {
      response->reply.status = cyberdog_visions_interfaces::msg::Reply::STATUS_ERROR;
      response->reply.status_msg = "status error, miloc is Not in navgating";
    } else {
      response->reply.status = cyberdog_visions_interfaces::msg::Reply::EXCEPTION;
      response->reply.status_msg = "reloc exception";

      response->pose.pose.position.x = position(0);
      response->pose.pose.position.y = position(1);
      response->pose.pose.position.z = position(2);

      response->pose.pose.orientation.x = orientation.x();
      response->pose.pose.orientation.y = orientation.y();
      response->pose.pose.orientation.z = orientation.z();
      response->pose.pose.orientation.w = orientation.w();
    }

  } else {
    img_lock_.unlock();
    response->reply.status = cyberdog_visions_interfaces::msg::Reply::FILE_ERROR;
    response->reply.status_msg = "No image recieved";
    response->reloc_id = request->reloc_id;
    response->is_verified = false;
    response->confidence = 0.0;
    return;
  }
}

void MilocServer::DeleteMapCallback(
  const std::shared_ptr<cyberdog_visions_interfaces::srv::MilocMapHandler::Request> request,
  const std::shared_ptr<cyberdog_visions_interfaces::srv::MilocMapHandler::Response> response)
{
  int map_id;
  map_id = request->map_id;
  if (0 == map_id) {
    map_id = 1;
  }
  int ret = miloc_api_->DeleteRelocMap(map_id);
  if (SLAM_OK == ret) {
    response->code = 0;
    response->message = "delete map success";
  } else {
    response->code = 100;
    response->message = "delete map fail";
  }
}

void MilocServer::GetMapStatusCallback(
  const std::shared_ptr<cyberdog_visions_interfaces::srv::MilocMapHandler::Request> request,
  const std::shared_ptr<cyberdog_visions_interfaces::srv::MilocMapHandler::Response> response)
{
  int map_id;
  map_id = request->map_id;
  if (0 == map_id) {
    map_id = 1;
  }
  int ret = miloc_api_->GetRelocMapStatus(map_id);
  response->code = ret;
  switch (ret) {
    case 0:
      response->message = "reloc map ok";
      break;
    case 300:
      response->message = "mapping";
      break;
    case 301:
      response->message = "reloc map unfinished";
      break;
    default:
      response->message = "reloc map error";
      break;
  }
}

void MilocServer::CreateMapCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (miloc_api_->GetMilocStatus() == MilocStatus::kRECONSTRUCTING) {
    response->success = false;
    response->message = "A reconstructing task is in progress, please wait for a moment";
    INFO("A reconstructing task is in progress, please wait for a moment, create map rejected");
    return;
  } else if (miloc_api_->GetMilocStatus() != MilocStatus::kINACTIVE) {
    ResetMiloc();
  }

  if (request->data) {
    while (!create_map_client_->wait_for_service(1s)) {
      INFO("Service not available, waiting again...");
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    create_map_client_->async_send_request(request);
  }


  Subscribe();

  miloc_api_->SetSLAM(request->data);
  if (request->data) {
    INFO("miloc work with visual slam");
  } else {
    INFO("miloc work with laser slam");
  }
  miloc_api_->CreateMap(current_map_id);
  create_map_id = current_map_id;

  response->success = true;
  response->message = "Begin to create map";
}

void MilocServer::FinishMapCallback(
  const std::shared_ptr<cyberdog_visions_interfaces::srv::FinishMap::Request> request,
  const std::shared_ptr<cyberdog_visions_interfaces::srv::FinishMap::Response> response)
{
  if (miloc_api_->GetMilocStatus() != MilocStatus::kMAPPABLE) {
    response->success = cyberdog_visions_interfaces::msg::Reply::STATUS_ERROR;
    response->message = "miloc Status ERROR!";
    return;
  }

  ResetMiloc();

  if (miloc_api_->GetSLAM()) {
    while (!finish_map_client_->wait_for_service(1s)) {
      INFO("Service not available, waiting again...");
    }
    auto request_map = std::make_shared<cyberdog_visions_interfaces::srv::FinishMap::Request>();
    request_map->finish = request->finish;
    request_map->map_name = request->map_name;
    finish_map_client_->async_send_request(request_map);
  }

  bool reconstruct_success = false;
  if (request->finish && !request->map_name.empty() && immediately_reconstruct_) {
    if (MilocStatus::kINACTIVE == miloc_api_->GetMilocStatus()) {
      if (ModelCheck() == SLAM_OK) {
        int ret = miloc_api_->ReconstructMap(create_map_id, "");
        create_map_id = -1;
        if (SLAM_OK == ret) {
          reconstruct_success = true;
          INFO("Start to reconstruct");
        } else {
          reconstruct_success = false;
          INFO("Something is wrong, reconstruction aborted");
        }
      }

    } else if (MilocStatus::kRECONSTRUCTING == miloc_api_->GetMilocStatus()) {
      reconstruct_success = false;
      INFO("Another reconstruction is proceeding, please wait for it to finish");
    } else if (MilocStatus::kMAPPABLE == miloc_api_->GetMilocStatus() ||
      MilocStatus::kMAPPING == miloc_api_->GetMilocStatus())
    {
      reconstruct_success = false;
      INFO("Miloc is in mapping mode");
    } else if (MilocStatus::kRELOCALIZABLE == miloc_api_->GetMilocStatus() ||
      MilocStatus::kRELOCATING == miloc_api_->GetMilocStatus())
    {
      reconstruct_success = false;
      INFO("Miloc is in relocalization mode");
    } else {
      reconstruct_success = false;
      INFO("Miloc is not initialized, please wait for a moment");
    }
  }

  response->success = reconstruct_success;
  response->message = "Finish map successfully";
}

void MilocServer::StartNavigationCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (miloc_api_->GetMilocStatus() == MilocStatus::kRECONSTRUCTING) {
    response->success = false;
    response->message = "A reconstructing task is in progress, please wait for a moment";
    INFO("A reconstructing task is in progress, please wait for a moment, start navigation rejected");
    return;
  } else if (miloc_api_->GetMilocStatus() == MilocStatus::kRELOCALIZABLE) {
    response->success = true;
    response->message = "MiLoc Status is Relocalizable already";
    return;
  } else if (miloc_api_->GetMilocStatus() != MilocStatus::kINACTIVE) {
    ResetMiloc();
  }

  Subscribe();
  fail_num_ = 0;
  miloc_api_->SetSLAM(request->data);
  if (ModelCheck() != SLAM_OK) {
    response->success = false;
    response->message = "Start to navigate error, model check error";
    INFO("Start to navigate error");
    return;
  }

  int ret = miloc_api_->StartNavigation();

  if (ret != SLAM_OK) {
    response->success = false;
    response->message = "Start to navigate error";
    INFO("Start to navigate error");
  } else {
    response->success = true;
    response->message = "Begin to navigate";
  }
}

void MilocServer::StopNavigationCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (miloc_api_->GetMilocStatus() != MilocStatus::kRELOCALIZABLE &&
    miloc_api_->GetMilocStatus() != MilocStatus::kRELOCATING)
  {
    response->success = cyberdog_visions_interfaces::msg::Reply::STATUS_ERROR;
    response->message = "miloc Status ERROR!";
    return;
  }

  ResetMiloc();
  response->success = true;
  if (request->data ^ miloc_api_->GetSLAM()) {
    response->message = "SLAM system is inconsistent";
  } else {
    response->message = "Stop navigating successfully";
  }
}

void MilocServer::CollectImage()
{
  using Clock = std::chrono::high_resolution_clock;
  using std::chrono::duration;
  using std::chrono::duration_cast;
  using std::chrono::milliseconds;

  std::vector<cv::Mat> images;
  std::vector<int> camera_idxs = {1, 2, 3};
  rclcpp::Rate loop_rate(5);
  auto last_interval = duration_cast<milliseconds>(Clock::now().time_since_epoch());

  const int MEAN_DURATION = 500;
  const int MIN_DURATION = 100;
  const int MAX_DURATION = 5000;
  auto time_to_sleep = std::chrono::milliseconds(MEAN_DURATION);
  while (rclcpp::ok()) {
    odom_lock_.lock();
    if (!odom_buffer_.empty()) {
      auto odom = odom_buffer_.back();
      Eigen::Vector3d v_linear(
        odom.twist.twist.linear.x,
        odom.twist.twist.linear.y,
        odom.twist.twist.linear.z);

      time_to_sleep = std::chrono::milliseconds(
        std::max(
          std::min(
            static_cast<int>(std::min(
              (MEAN_DURATION / (v_linear.norm() / 0.5)),
              (MEAN_DURATION / (std::fabs(odom.twist.twist.angular.z) / (M_PI / 6))))),
            MAX_DURATION),
          MIN_DURATION));
    } else {
      time_to_sleep = std::chrono::milliseconds(MEAN_DURATION);
    }
    odom_lock_.unlock();

    auto new_interval = duration_cast<milliseconds>(Clock::now().time_since_epoch());
    auto period = new_interval - last_interval;
    if (time_to_sleep.count() > period.count()) {
      loop_rate.sleep();
      continue;
    }
    last_interval = new_interval;

    ImgData img_data;
    img_lock_.lock();
    if (!img_buffer_.empty()) {
      img_data = img_buffer_.front();
      img_buffer_.pop_front();
    } else {
      img_lock_.unlock();
      loop_rate.sleep();
      continue;
    }
    img_lock_.unlock();

    images.clear();
    images.push_back(img_data.img_front);
    images.push_back(img_data.img_left);
    images.push_back(img_data.img_right);
    miloc_api_->CollectImage(img_data.time_stamp, images, camera_idxs);
    loop_rate.sleep();
  }
}

}  // namespace miloc
}  // namespace cyberdog

RCLCPP_COMPONENTS_REGISTER_NODE(cyberdog::miloc::MilocServer)

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto node = std::make_shared<cyberdog::miloc::MilocServer>();
  exec.add_node(node);

  exec.spin();
  node->ShutDown();
  rclcpp::shutdown();

  return 0;
}
