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
  using namespace std::chrono_literals;
  heartbeat_timer_ = create_wall_timer(
    100ms, std::bind(&TechnicalDirectorNetworkBridgeComponent::publishHeartBeat, this));
}

void TechnicalDirectorNetworkBridgeComponent::publishHeartBeat() {}
}  // namespace robotx_communication

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(robotx_communication::TechnicalDirectorNetworkBridgeComponent)