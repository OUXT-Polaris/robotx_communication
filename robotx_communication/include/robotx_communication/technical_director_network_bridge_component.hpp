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

#ifndef ROBOTX_COMMUNICATION__TECHNICAL_DIRECTOR_NETWORK_BRIDGE_COMPONENT_H_
#define ROBOTX_COMMUNICATION__TECHNICAL_DIRECTOR_NETWORK_BRIDGE_COMPONENT_H_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ROBOTX_COMMUNICATION_TECHNICAL_DIRECTOR_NETWORK_BRIDGE_COMPONENT_EXPORT \
  __attribute__((dllexport))
#define ROBOTX_COMMUNICATION_TECHNICAL_DIRECTOR_NETWORK_BRIDGE_COMPONENT_IMPORT \
  __attribute__((dllimport))
#else
#define ROBOTX_COMMUNICATION_TECHNICAL_DIRECTOR_NETWORK_BRIDGE_COMPONENT_EXPORT \
  __declspec(dllexport)
#define ROBOTX_COMMUNICATION_TECHNICAL_DIRECTOR_NETWORK_BRIDGE_COMPONENT_IMPORT \
  __declspec(dllimport)
#endif
#ifdef ROBOTX_COMMUNICATION_TECHNICAL_DIRECTOR_NETWORK_BRIDGE_COMPONENT_BUILDING_DLL
#define ROBOTX_COMMUNICATION_TECHNICAL_DIRECTOR_NETWORK_BRIDGE_COMPONENT_PUBLIC \
  ROBOTX_COMMUNICATION_TECHNICAL_DIRECTOR_NETWORK_BRIDGE_COMPONENT_EXPORT
#else
#define ROBOTX_COMMUNICATION_TECHNICAL_DIRECTOR_NETWORK_BRIDGE_COMPONENT_PUBLIC \
  ROBOTX_COMMUNICATION_TECHNICAL_DIRECTOR_NETWORK_BRIDGE_COMPONENT_IMPORT
#endif
#define ROBOTX_COMMUNICATION_TECHNICAL_DIRECTOR_NETWORK_BRIDGE_COMPONENT_PUBLIC_TYPE \
  ROBOTX_COMMUNICATION_TECHNICAL_DIRECTOR_NETWORK_BRIDGE_COMPONENT_PUBLIC
#define ROBOTX_COMMUNICATION_TECHNICAL_DIRECTOR_NETWORK_BRIDGE_COMPONENT_LOCAL
#else
#define ROBOTX_COMMUNICATION_TECHNICAL_DIRECTOR_NETWORK_BRIDGE_COMPONENT_EXPORT \
  __attribute__((visibility("default")))
#define ROBOTX_COMMUNICATION_TECHNICAL_DIRECTOR_NETWORK_BRIDGE_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define ROBOTX_COMMUNICATION_TECHNICAL_DIRECTOR_NETWORK_BRIDGE_COMPONENT_PUBLIC \
  __attribute__((visibility("default")))
#define ROBOTX_COMMUNICATION_TECHNICAL_DIRECTOR_NETWORK_BRIDGE_COMPONENT_LOCAL \
  __attribute__((visibility("hidden")))
#else
#define ROBOTX_COMMUNICATION_TECHNICAL_DIRECTOR_NETWORK_BRIDGE_COMPONENT_PUBLIC
#define ROBOTX_COMMUNICATION_TECHNICAL_DIRECTOR_NETWORK_BRIDGE_COMPONENT_LOCAL
#endif
#define ROBOTX_COMMUNICATION_TECHNICAL_DIRECTOR_NETWORK_BRIDGE_COMPONENT_PUBLIC_TYPE
#endif
#if __cplusplus
}  // extern "C"
#endif

#include <geographic_msgs/msg/geo_point.hpp>
#include <rclcpp/rclcpp.hpp>

namespace robotx_communication
{
class TechnicalDirectorNetworkBridgeComponent : public rclcpp::Node
{
public:
  ROBOTX_COMMUNICATION_TECHNICAL_DIRECTOR_NETWORK_BRIDGE_COMPONENT_PUBLIC
  TechnicalDirectorNetworkBridgeComponent(const rclcpp::NodeOptions & options);

private:
  void publishHeartBeat();
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};
}  // namespace robotx_communication

#endif  // ROBOTX_COMMUNICATION__TECHNICAL_DIRECTOR_NETWORK_BRIDGE_COMPONENT_H_
