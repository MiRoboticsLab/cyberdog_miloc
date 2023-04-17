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

#ifndef CYBERDOG_MILOC__MILOC_SERVER_HPP_
#define CYBERDOG_MILOC__MILOC_SERVER_HPP_

#include <thread>
#include <deque>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int32.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "cyberdog_visions_interfaces/msg/miloc_status.hpp"
#include "cyberdog_visions_interfaces/srv/reloc.hpp"
#include "cyberdog_visions_interfaces/srv/miloc_map_handler.hpp"
#include "cyberdog_visions_interfaces/srv/map_config.hpp"
#include "cyberdog_visions_interfaces/srv/map_param.hpp"
#include "cyberdog_visions_interfaces/srv/finish_map.hpp"
#include "cyberdog_common/cyberdog_model.hpp"
#include "protocol/srv/map.hpp"
#include "protocol/msg/connector_status.hpp"

#include "api/miloc_api.hpp"
#include "miloc_types.hpp"

namespace cyberdog
{
namespace miloc
{
typedef message_filters::Subscriber<sensor_msgs::msg::Image> ImageSyncSub;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
    sensor_msgs::msg::Image,
    sensor_msgs::msg::Image> ImageSyncPolicyTriple;
typedef message_filters::Synchronizer<ImageSyncPolicyTriple> ImageSyncTriple;

/**
 * @brief miloc_server node class 
 * 
 */
class MilocServer : public rclcpp::Node
{
public:
  explicit MilocServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("MilocServer", options)
  {
    img_buffer_size_ = 5;
    pose_buffer_size_ = 10;
    odom_buffer_size_ = 10;
    Init();
  }
  ~MilocServer() = default;

  int Init();

  int ShutDown();

  int Subscribe();

  int Unsubscribe();

  void ResetMiloc();

private:
  void MilocStatusCallback();
  /**
   * @brief Subscribe Triple Camreas
   * 
   * @param msg_front 
   * @param msg_left 
   * @param msg_right 
   */
  void TripleCameraCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr msg_front,
    const sensor_msgs::msg::Image::ConstSharedPtr msg_left,
    const sensor_msgs::msg::Image::ConstSharedPtr msg_right);

  void OdometryCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr msg_odom);

  void OdometryLegCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr msg_odom);
  /**
   * @brief reloc interface, response the robot pose
   */
  void RelocCallback(
    const std::shared_ptr<cyberdog_visions_interfaces::srv::Reloc::Request> request,
    const std::shared_ptr<cyberdog_visions_interfaces::srv::Reloc::Response> response);
  /**
   * @brief change miloc status to mappble and start to collect images
   */
  void CreateMapCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  /**
   * @brief stop collecting images and start to reconstruct reloc map
   */
  void FinishMapCallback(
    const std::shared_ptr<cyberdog_visions_interfaces::srv::FinishMap::Request> request,
    const std::shared_ptr<cyberdog_visions_interfaces::srv::FinishMap::Response> response);
  /**
   * @brief preparing for reloc include load reloc map, reloc models and subscribe images
   */
  void StartNavigationCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  /**
   * @brief release map, models and unsubscribe images
   */
  void StopNavigationCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  /**
   * @brief delete reloc map interface
   */
  void DeleteMapCallback(
    const std::shared_ptr<cyberdog_visions_interfaces::srv::MilocMapHandler::Request> request,
    const std::shared_ptr<cyberdog_visions_interfaces::srv::MilocMapHandler::Response> response);
  /**
   * @brief check reloc map is available or not
   */
  void GetMapStatusCallback(
    const std::shared_ptr<cyberdog_visions_interfaces::srv::MilocMapHandler::Request> request,
    const std::shared_ptr<cyberdog_visions_interfaces::srv::MilocMapHandler::Response> response);
  /**
   * @brief download reloc models online
   */
  void ModelDownloadCallback(const protocol::msg::ConnectorStatus::SharedPtr msg);

  void CollectImage();

  void CollectPose();
  /**
   * @brief check the model version is available or not
   * 
   * @return 1 for check success
   * @return 0 for check fail 
   */
  int ModelCheck();

  std::deque<ImgData> img_buffer_;
  std::deque<geometry_msgs::msg::PoseStamped> pose_buffer_;
  std::deque<nav_msgs::msg::Odometry> odom_buffer_;
  uint64_t img_buffer_size_;
  uint64_t pose_buffer_size_;
  uint64_t odom_buffer_size_;
  std::mutex img_lock_;
  std::mutex pose_lock_;
  std::mutex odom_lock_;
  bool first_reloc_success_ = false;
  int fail_num_ = 0;
  int reloc_failure_threshold_;
  bool immediately_reconstruct_;
  std::shared_ptr<cyberdog::common::cyberdog_model> miloc_model_;

  rclcpp::Subscription<protocol::msg::ConnectorStatus>::SharedPtr connector_sub_;
  std::vector<std::unique_ptr<ImageSyncSub>> sync_subs_cam_;
  std::vector<std::unique_ptr<ImageSyncTriple>> sync_cam_triple_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_leg_subscriber_;

  std::unique_ptr<std::thread> collect_image_thread_;

  // Topics
  rclcpp::TimerBase::SharedPtr miloc_status_timer_;
  rclcpp::Publisher<cyberdog_visions_interfaces::msg::MilocStatus>::SharedPtr
    miloc_status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr reloc_result_publisher_;

  // Services
  rclcpp::Service<cyberdog_visions_interfaces::srv::Reloc>::SharedPtr reloc_service_;
  rclcpp::Service<cyberdog_visions_interfaces::srv::MapConfig>::SharedPtr set_map_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr create_map_service_;
  // rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr finish_map_service_;
  rclcpp::Service<cyberdog_visions_interfaces::srv::FinishMap>::SharedPtr finish_map_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_nav_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr stop_nav_service_;

  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr create_map_client_;
  rclcpp::Client<cyberdog_visions_interfaces::srv::FinishMap>::SharedPtr finish_map_client_;

  rclcpp::Service<cyberdog_visions_interfaces::srv::MilocMapHandler>::SharedPtr get_miloc_status_;
  rclcpp::Service<cyberdog_visions_interfaces::srv::MilocMapHandler>::SharedPtr delete_map_service_;

  std::shared_ptr<MilocApi> miloc_api_;

  int current_map_id_;
  int create_map_id_;
};  // class MilocServer

}  // namespace miloc
}  // namespace cyberdog

#endif  // CYBERDOG_MILOC__MILOC_SERVER_HPP_
