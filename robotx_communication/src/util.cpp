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
std::vector<std::byte> toBytesArray(const std::string & str)
{
  std::vector<std::byte> bytes;
  std::transform(
    std::begin(str), std::end(str), std::begin(bytes), [](char c) { return std::byte(c); });
  return bytes;
}

std::byte bitxor(const std::string & str)
{
  const auto bytes = toBytesArray(str);
  /*
  std::byte checksum = str.at(0);
  for (size_t i = 1; i < str.length(); i++) {
    checksum = checksum ^ str.at(i);
  }
  return checksum;
  */
  std::byte a{0b1010'1010};
  return a;
}
}  // namespace robotx_communication
