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

#ifndef ROBOTX_COMMUNICATION__SUBSCRIBER_HPP_
#define ROBOTX_COMMUNICATION__SUBSCRIBER_HPP_

#include <boost/optional.hpp>
#include <rclcpp/rclcpp.hpp>

namespace robotx_communication
{
template <class T>
class Subscriber
{
public:
  template <class NodeT, class AllocatorT = std::allocator<void>>
  Subscriber(
    NodeT && node, const std::string & topic, double timeout_duration = 1.0,
    const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort())
  : topic(topic),
    timeout_duration(timeout_duration),
    sub_(rclcpp::create_subscription<T>(
      node, topic, qos, std::bind(&Subscriber::callback, this, std::placeholders::_1),
      rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>())),
    clock_(node->get_clock()),
    logger_(node->get_logger())
  {
  }
  bool isTimeout()
  {
    if ((clock_->now() - last_timestamp_).seconds() >= timeout_duration) {
      return true;
    }
    RCLCPP_WARN_STREAM(logger_, "topic : " << topic << " is timeout.");
    return false;
  }

  boost::optional<std::shared_ptr<T>> get()
  {
    if (!data_) {
      RCLCPP_WARN_STREAM(logger_, "topic : " << topic << " has not subscribed yet.");
    }
    return data_;
  }
  const std::string topic;
  const double timeout_duration;

private:
  void callback(const std::shared_ptr<T> msg)
  {
    data_ = msg;
    last_timestamp_ = clock_->now();
  }
  boost::optional<std::shared_ptr<T>> data_;
  const std::shared_ptr<rclcpp::Subscription<T>> sub_;
  const std::shared_ptr<rclcpp::Clock> clock_;
  const rclcpp::Logger logger_;
  rclcpp::Time last_timestamp_;
};
}  // namespace robotx_communication

#endif  // ROBOTX_COMMUNICATION__SUBSCRIBER_HPP_