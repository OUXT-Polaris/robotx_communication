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

#ifndef ROBOTX_COMMUNICATION__UTIL_HPP_
#define ROBOTX_COMMUNICATION__UTIL_HPP_

#include <algorithm>
#include <geographic_msgs/msg/geo_point.hpp>
#include <string>
#include <vector>

namespace robotx_communication
{
const std::string & getHexString(uint8_t value);
const std::string & bitxor(const std::string & str);
const std::string & getDateTimeString();
const std::string & floatToString(float f, int digits);
const std::string & getGeoPointString(
  const std::shared_ptr<geographic_msgs::msg::GeoPoint> & geo_point);
}  // namespace robotx_communication

#endif  // ROBOTX_COMMUNICATION__UTIL_HPP_
