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
std::string getHexString(uint8_t value)
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

std::string bitxor(const std::string & str)
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
}  // namespace robotx_communication
