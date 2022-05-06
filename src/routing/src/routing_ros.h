/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <memory>

#include "../../../src/routing/src/routing.h"
#include "../../../cyber/common/file.h"

//ROS msgs
//#include "../../../devel/include/routing_msgs/RosRoutingRequest.h"
#include "routing_msgs/LaneWaypoint.h"
#include "routing_msgs/RosRoutingRequest.h"

#include <ros/ros.h>
#include <iostream>

namespace apollo {
namespace routing {

class RoutingRos {
 public:
  //RoutingRos() = default;
  //~RoutingRos() = default;
  RoutingRos() {}
  ~RoutingRos() {}
 public:
  bool Init();

  bool start();

private:
  void RoutingCallBack(const routing_msgs::RosRoutingRequest req);
  //apolloCallBack:收到routing request的时候触发
  bool Proc(const std::shared_ptr<RoutingRequest>& request);

  private:
 //routing消息发布handle
/*
  std::shared_ptr<::apollo::cyber::Writer<RoutingResponse>> response_writer_ =
      nullptr;
  std::shared_ptr<::apollo::cyber::Writer<RoutingResponse>>
      response_history_writer_ = nullptr;
*/

  Routing routing_;
  ros::Publisher resPub;
  ros::Subscriber reqSub;
  ros::NodeHandle nh;
/*
  std::shared_ptr<RoutingResponse> response_ = nullptr;
  //定时器
  std::unique_ptr<::apollo::cyber::Timer> timer_;
  //锁
  std::mutex mutex_;
*/
};


//在cyber框架中注册routing模块
//CYBER_REGISTER_COMPONENT(RoutingComponent)

}  // namespace routing
}  // namespace apollo
