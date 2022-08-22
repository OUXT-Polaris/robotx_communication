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
: Node("technical_director_network_bridge", options)
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

  /**
   * @brief setup callbacks for heart beat
   */
  setupGeoPoint();
  setupAmsStatus();
  setupUavStatus();
  using namespace std::chrono_literals;
  heartbeat_timer_ = create_wall_timer(
    100ms, std::bind(&TechnicalDirectorNetworkBridgeComponent::publishHeartBeat, this));
}

void TechnicalDirectorNetworkBridgeComponent::setupGeoPoint()
{
  declare_parameter<std::string>("geo_point_topic", "geo_point");
  std::string geo_point_topic = get_parameter("geo_point").as_string();
  geo_point_sub_ = create_subscription<geographic_msgs::msg::GeoPoint>(
    geo_point_topic, 1,
    std::bind(
      &TechnicalDirectorNetworkBridgeComponent::geoPointCallback, this, std::placeholders::_1));
}

void TechnicalDirectorNetworkBridgeComponent::setupAmsStatus()
{
  declare_parameter<std::string>("autonomous_maritime_system_status_topic", "ams/status");
  std::string autonomous_maritime_system_status_topic =
    get_parameter("autonomous_maritime_system_status_topic").as_string();
  autonomous_maritime_system_status_sub_ =
    create_subscription<robotx_msgs::msg::AutonomousMaritimeSystemStatus>(
      autonomous_maritime_system_status_topic, 1,
      std::bind(
        &TechnicalDirectorNetworkBridgeComponent::autonomousMaritimeSystemStatusCallback, this,
        std::placeholders::_1));
}

void TechnicalDirectorNetworkBridgeComponent::setupUavStatus()
{
  declare_parameter<std::string>("unmanned_aerial_vehicle_status_topic", "uav/status");
  std::string unmanned_aerial_vehicle_status_topic =
    get_parameter("unmanned_aerial_vehicle_status_topic").as_string();
  unmanned_aerial_vehicle_status_sub_ =
    create_subscription<robotx_msgs::msg::UnmannedAerialVehicleStatus>(
      unmanned_aerial_vehicle_status_topic, 1,
      std::bind(
        &TechnicalDirectorNetworkBridgeComponent::unmannedAerialVehicleStatusCallback, this,
        std::placeholders::_1));
}

void TechnicalDirectorNetworkBridgeComponent::geoPointCallback(
  const geographic_msgs::msg::GeoPoint::SharedPtr msg)
{
  geo_point_ = msg;
}

void TechnicalDirectorNetworkBridgeComponent::autonomousMaritimeSystemStatusCallback(
  const robotx_msgs::msg::AutonomousMaritimeSystemStatus::SharedPtr msg)
{
  autonomous_maritime_system_status_ = msg;
}

void TechnicalDirectorNetworkBridgeComponent::unmannedAerialVehicleStatusCallback(
  const robotx_msgs::msg::UnmannedAerialVehicleStatus::SharedPtr msg)
{
  unmanned_aerial_vehicle_status_ = msg;
}

void TechnicalDirectorNetworkBridgeComponent::publishHeartBeat()
{
  if (!geo_point_) {
    RCLCPP_WARN_STREAM(get_logger(), "geopoint has not recieved yet");
    return;
  }
  if (!autonomous_maritime_system_status_) {
    RCLCPP_WARN_STREAM(get_logger(), "autonomous_maritime_system_status has not recieved yet");
  }
  if (!unmanned_aerial_vehicle_status_) {
    RCLCPP_WARN_STREAM(get_logger(), "unmanned_aerial_vehicle_status has not recieved yet");
  }
  std::string msg = "$RXHRB";
  rclcpp::Time now = get_clock()->now();
}
}  // namespace robotx_communication

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(robotx_communication::TechnicalDirectorNetworkBridgeComponent)
