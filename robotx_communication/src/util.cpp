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
#include <robotx_communication/util.hpp>

namespace robotx_communication
{
const std::string & getHexString(uint8_t value)
{
  if (value > 16) {
    throw std::runtime_error("value is over 16, current value is " + std::to_string(value));
  }
  std::string ret;
  if (value == 10) {
    ret = "A";
  } else if (value == 11) {
    ret = "B";
  } else if (value == 12) {
    ret = "C";
  } else if (value == 13) {
    ret = "D";
  } else if (value == 14) {
    ret = "E";
  } else if (value == 15) {
    ret = "F";
  } else {
    ret = std::to_string(value);
  }
  return ret;
}

const std::string & bitxor(const std::string & str)
{
  uint8_t checksum = 0;
  for (size_t i = 1; i < str.size(); i++) {
    int c = str[i];
    checksum ^= c;
  }
  uint8_t rest = checksum % 16;
  uint8_t quotient = (checksum - rest) / 16;
  std::string ret = getHexString(quotient) + getHexString(rest);
  return ret;
}

const std::string & getDateTimeString()
{
  auto timer = time(NULL);
  const auto local = localtime(&timer);
  std::string year = std::to_string(local->tm_year + 1900);
  year.substr(year.length() - 2, 2);
  std::string month = std::to_string(local->tm_mon + 1);
  if (local->tm_mon <= 9) {
    month = "0" + month;
  }
  std::string day = std::to_string(local->tm_mday);
  if (local->tm_mday <= 9) {
    day = "0" + day;
  }
  std::string hour = std::to_string(local->tm_hour);
  if (local->tm_hour <= 9) {
    hour = "0" + hour;
  }
  std::string minutes = std::to_string(local->tm_min);
  if (local->tm_min <= 9) {
    minutes = "0" + minutes;
  }
  std::string seconds = std::to_string(local->tm_sec);
  if (local->tm_sec <= 9) {
    seconds = "0" + seconds;
  }
  std::string ret = day + month + year + "," + hour + minutes + seconds;
  return ret;
}
}  // namespace robotx_communication
