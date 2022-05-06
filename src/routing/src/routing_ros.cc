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
/*
#include "modules/routing/routing_component.h"

#include <utility>

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/routing/common/routing_gflags.h"
*/

#include "../../../src/routing/src/routing_ros.h"

#include <utility>

//#include "modules/common/adapters/adapter_gflags.h"
#include "../../../src/routing/common/routing_gflags.h"

//DECLARE_string(flagfile);

namespace apollo {
namespace routing {

bool RoutingRos::Init() {
  RoutingConfig routing_conf;
/*
  ACHECK(cyber::ComponentBase::GetProtoConfig(&routing_conf))
      << "Unable to load routing conf file: "
      << cyber::ComponentBase::ConfigFilePath();

  AINFO << "Config file: " << cyber::ComponentBase::ConfigFilePath()
        << " is loaded.";
*/
  std::string routing_config_file=
      "/home/casicapollo/Documents/myAll/my_3_3_1/src/routing/conf/routing_config.pb.txt";  
  if(!apollo::cyber::common::GetProtoFromFile(routing_config_file,&routing_conf))
  {
    std::cout<<"Unable to load routing conf file: "
              <<routing_config_file<<std::endl;
  }
  else
  {
    std::cout<<"Config file: "
            <<routing_config_file<< " is loaded."<<std::endl;
  }

/*
  // 设置消息qos，控制流量，创建消息发布response_writer_
  apollo::cyber::proto::RoleAttributes attr;
  attr.set_channel_name(routing_conf.topic_config().routing_response_topic());
  auto qos = attr.mutable_qos_profile();
  qos->set_history(apollo::cyber::proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
  qos->set_reliability(
      apollo::cyber::proto::QosReliabilityPolicy::RELIABILITY_RELIABLE);
  qos->set_durability(
      apollo::cyber::proto::QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL);
  response_writer_ = node_->CreateWriter<RoutingResponse>(attr);

  apollo::cyber::proto::RoleAttributes attr_history;
  attr_history.set_channel_name(
      routing_conf.topic_config().routing_response_history_topic());
  auto qos_history = attr_history.mutable_qos_profile();
  qos_history->set_history(
      apollo::cyber::proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
  qos_history->set_reliability(
      apollo::cyber::proto::QosReliabilityPolicy::RELIABILITY_RELIABLE);
  qos_history->set_durability(
      apollo::cyber::proto::QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL);

  response_history_writer_ = node_->CreateWriter<RoutingResponse>(attr_history);

  //创建定时器
  std::weak_ptr<RoutingComponent> self =
      std::dynamic_pointer_cast<RoutingComponent>(shared_from_this());
  timer_.reset(new ::apollo::cyber::Timer(
      FLAGS_routing_response_history_interval_ms,
      [self, this]() {
        auto ptr = self.lock();
        if (ptr) {
          std::lock_guard<std::mutex> guard(this->mutex_);
          if (this->response_ != nullptr) {
            auto timestamp = apollo::cyber::Clock::NowInSeconds();
            response_->mutable_header()->set_timestamp_sec(timestamp);
            this->response_history_writer_->Write(*response_);
          }
        }
      },
      false));
  timer_->Start();
*/

  //执行routing类
  return routing_.Init().ok() && routing_.Start().ok();
}

bool RoutingRos::start()
{
    std::cout<<"RoutingRos::start"<<std::endl;
    //sub
    reqSub=nh.subscribe("/routingrequest",1000,&RoutingRos::RoutingCallBack,this);
    //pub
    //resPub=nh.advertise<routing_msgs::RosRoutingRequest>("/waiting0",10);
    return true;
}

void RoutingRos::RoutingCallBack(const routing_msgs::RosRoutingRequest req)
{
  std::cout<<"RoutingRos::RoutingCallBack"<<std::endl;

  //???????????????????????
  std::shared_ptr<RoutingRequest> re=
                    std::make_shared<RoutingRequest>();

  std::cout<<"before proc0"<<std::endl; 
  for(auto waypoint_:req.waypoint)
  {
    LaneWaypoint* lwpoint=re->add_waypoint();
    lwpoint->set_id(waypoint_.id);
    lwpoint->set_s(waypoint_.s);

    common::PointENU *penu=lwpoint->mutable_pose();
    penu->set_x(waypoint_.x);
    penu->set_y(waypoint_.y);
    penu->set_z(waypoint_.z);    
  }
  std::cout<<"before proc1"<<std::endl;

    for(auto wp:re->waypoint())
    {
    ROS_INFO("publish routing_msgs0 command  [%f] [%f] [%f] ",            
             wp.pose().x(),
             wp.pose().y(),
             wp.pose().z());      
    }
  //
  Proc(re);
}


//routing模块接收到routing_request时，触发Proc，返回routing_response
bool RoutingRos::Proc(const std::shared_ptr<RoutingRequest>& request) 
{
  std::cout<<"RoutingRos::Proc"<<std::endl;

  auto response = std::make_shared<RoutingResponse>();
  //响应routing_请求
  if (!routing_.Process(request, response.get())) {
    return false;
  }
  //填充头部信息，并发布
  /*
  common::util::FillHeader(node_->Name(), response.get());
  response_writer_->Write(response);
  {
    std::lock_guard<std::mutex> guard(mutex_);
    response_ = std::move(response);
  }
  */
  return true;
}

}  // namespace routing
}  // namespace apollo
