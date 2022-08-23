// Copyright (c) 2022 OUXT Polaris
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

#include <robotx_communication/technical_director_network_bridge_component.hpp>

namespace robotx_communication
{
TechnicalDirectorNetworkBridgeComponent::TechnicalDirectorNetworkBridgeComponent(
  const rclcpp::NodeOptions & options)
: Node("technical_director_network_bridge", options),
  geo_sub_(this, "geo_point"),
  uav_status_sub_(this, "uav/status"),
  ams_status_sub_(this, "ams/status")
{
  /**
   * @brief setup parameters
   */
  declare_parameter("team_id", "OUXTPOLARIS");
  team_id_ = get_parameter("team_id").as_string();
  /**
   * @brief setup tcp client
   */
  declare_parameter<std::string>("ip_address", "0.0.0.0");
  std::string ip_address = get_parameter("ip_address").as_string();
  declare_parameter<int>("port", 8000);
  int port = get_parameter("port").as_int();
  tcp_client_ = std::make_unique<tcp_sender::TcpClient>(io_service_, get_logger());
  tcp_client_->connect(ip_address, port);

  using namespace std::chrono_literals;
  heartbeat_timer_ = create_wall_timer(
    100ms, std::bind(&TechnicalDirectorNetworkBridgeComponent::publishHeartBeat, this));
}

uint8_t TechnicalDirectorNetworkBridgeComponent::bitxor(const std::string & str) const
{
  unsigned char checksum = str.at(0);
  for (size_t i = 1; i < str.length(); i++) {
    checksum = checksum ^ str.at(i);
  }
  return checksum;
}

void TechnicalDirectorNetworkBridgeComponent::publishHeartBeat()
{
  if (!geo_sub_.isTimeout() || !ams_status_sub_.isTimeout() || !uav_status_sub_.isTimeout()) {
    return;
  }
  const auto geo_point = geo_sub_.get();
  const auto ams_status = ams_status_sub_.get();
  const auto uav_status = uav_status_sub_.get();
  if (!geo_point || !ams_status || !uav_status) {
    return;
  }
  std::string msg = "$RXHRB";
  rclcpp::Time now = get_clock()->now();
}
}  // namespace robotx_communication

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(robotx_communication::TechnicalDirectorNetworkBridgeComponent)
