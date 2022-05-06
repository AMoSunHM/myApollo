/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include <limits>
#include <unordered_map>
/*
#include "modules/routing/routing.h"

#include "modules/common/util/point_factory.h"
#include "modules/routing/common/routing_gflags.h"
*/
#include "../../../src/routing/src/routing.h"

#include "../../../src/common/util/point_factory.h"
#include "../../../src/routing/common/routing_gflags.h"

namespace apollo {
namespace routing {

using apollo::common::ErrorCode;
using apollo::common::PointENU;
using apollo::hdmap::ParkingSpaceInfoConstPtr;

std::string Routing::Name() const { return FLAGS_routing_node_name; }

//Routing::Routing()
//    : monitor_logger_buffer_(common::monitor::MonitorMessageItem::ROUTING) {}

apollo::common::Status Routing::Init() {
  //读取routing_map，也就是点和边
  const auto routing_map_file = apollo::hdmap::RoutingMapFile();
/*  
  const auto routing_map_file = 
        "/home/casicapollo/Documents/myAll/my_3_3_1/src/map/data/routing_map.txt";
*/

  AINFO << "Use routing topology graph path: " << routing_map_file;
  /*
  std::cout<< "Use routing topology graph path: " 
            << routing_map_file<<std::endl;
  */
  navigator_ptr_.reset(new Navigator(routing_map_file));

  //读取高精地图，用来查找routing_request请求的点距离最近的lane
  //并且返回对应的lane_id
  //比如在小区里打车，需要找到最近的乘车点，就是找到最近的路
  hdmap_ = apollo::hdmap::HDMapUtil::BaseMapPtr();

  ACHECK(hdmap_) << "Failed to load map file:" << apollo::hdmap::BaseMapFile();
/*
  if(hdmap_==nullptr) {
    std::cout<<"Failed to load map file:" << apollo::hdmap::BaseMapFile()<<std::endl;
  }
*/
  return apollo::common::Status::OK();
}

apollo::common::Status Routing::Start() {
  if (!navigator_ptr_->IsReady()) {
    AERROR << "Navigator is not ready!";
    //std::cout<<"AERROR "<< "Navigator is not ready!"<<std::endl;
    return apollo::common::Status(ErrorCode::ROUTING_ERROR,
                                  "Navigator not ready");
  }
  AINFO << "Routing service is ready.";
  //std::cout << "Routing service is ready." <<std::endl;

  //monitor_logger_buffer_.INFO("Routing started");
  return apollo::common::Status::OK();
}

std::vector<RoutingRequest> Routing::FillLaneInfoIfMissing(
    const RoutingRequest& routing_request) {
  std::vector<RoutingRequest> fixed_requests;
  std::unordered_map<int, std::vector<LaneWaypoint>>
      additional_lane_waypoint_map;
  RoutingRequest fixed_request(routing_request);
  for (int i = 0; i < routing_request.waypoint_size(); ++i) {
    LaneWaypoint lane_waypoint(routing_request.waypoint(i));
    //从request中获取的点是没有id信息，需要填充为搜索到的所在车道id
    if (lane_waypoint.has_id()) {
      continue;
    }
    // fill lane info when missing
    // 需补充信息包含所在车道id(lane_id)，点所处车道的位置信息(s)
    // 将request中的waypoint点提出，生成一个内部定义的数据结构
    const auto point =
        common::util::PointFactory::ToPointENU(lane_waypoint.pose());
    std::vector<std::shared_ptr<const hdmap::LaneInfo>> lanes;
    // look for lanes with bigger radius if not found
    //防止选择的点距离车道线较远，无法匹配，增加半径为6米的范围冗余
    //返回point为圆形，(kRadius+i*kRadius)为半径范围内存在车道线的信息
    constexpr double kRadius = 0.3;
    for (int i = 0; i < 20; ++i) {
      //此处通过已经建立好的kdtree，快速搜索满足条件的车道信息(lanes)
      hdmap_->GetLanes(point, kRadius + i * kRadius, &lanes);
      if (lanes.size() > 0) {
        break;
      }
    }
    if (lanes.empty()) {

      AERROR << "Failed to find nearest lane from map at position: "
             << point.DebugString();
      /*
      std::cout<<"AERROR "<< "Failed to find nearest lane from map at position: "
             << point.DebugString()<<std::endl;
      */
      return fixed_requests;  // return empty vector
    }
    for (size_t j = 0; j < lanes.size(); ++j) {
      double s = 0.0;
      double l = 0.0;
      lanes[j]->GetProjection({point.x(), point.y()}, &s, &l);
      if (j == 0) {
        auto waypoint_info = fixed_request.mutable_waypoint(i);
        waypoint_info->set_id(lanes[j]->id().id());
        waypoint_info->set_s(s);
      } else {
        // additional candidate lanes
        LaneWaypoint new_lane_waypoint(lane_waypoint);
        new_lane_waypoint.set_id(lanes[j]->id().id());
        new_lane_waypoint.set_s(s);
        additional_lane_waypoint_map[i].push_back(new_lane_waypoint);
      }
    }
  }
  // first routing_request
  fixed_requests.push_back(fixed_request);

  // additional routing_requests because of lane overlaps
  for (const auto& m : additional_lane_waypoint_map) {
    size_t cur_size = fixed_requests.size();
    for (size_t i = 0; i < cur_size; ++i) {
      // use index to iterate while keeping push_back
      for (const auto& lane_waypoint : m.second) {
        RoutingRequest new_request(fixed_requests[i]);
        auto waypoint_info = new_request.mutable_waypoint(m.first);
        waypoint_info->set_id(lane_waypoint.id());
        waypoint_info->set_s(lane_waypoint.s());
        fixed_requests.push_back(new_request);
      }
    }
  }

  for (const auto& fixed_request : fixed_requests) {
    ADEBUG << "Fixed routing request:" << fixed_request.DebugString();
    /*
    std::cout<<"ADEBUG "<< "Fixed routing request:" 
            << fixed_request.DebugString()<<std::endl;
    */
  }
  return fixed_requests;
}

bool Routing::GetParkingID(const PointENU& parking_point,
                           std::string* parking_space_id) {
  // search current parking space id associated with parking point.
  constexpr double kDistance = 0.01;  // meter
  std::vector<ParkingSpaceInfoConstPtr> parking_spaces;
  if (hdmap_->GetParkingSpaces(parking_point, kDistance, &parking_spaces) ==
      0) {
    *parking_space_id = parking_spaces.front()->id().id();
    return true;
  }
  return false;
}

bool Routing::FillParkingID(RoutingResponse* routing_response) {
  const auto& routing_request = routing_response->routing_request();
  const bool has_parking_info = routing_request.has_parking_info();
  const bool has_parking_id =
      has_parking_info && routing_request.parking_info().has_parking_space_id();
  // return early when has parking_id
  if (has_parking_id) {
    return true;
  }
  // set parking space ID when
  //  has parking info && has parking point && NOT has parking space id && get
  //  ID successfully
  if (has_parking_info && routing_request.parking_info().has_parking_point()) {
    const PointENU parking_point =
        routing_request.parking_info().parking_point();
    std::string parking_space_id;
    if (GetParkingID(parking_point, &parking_space_id)) {
      routing_response->mutable_routing_request()
          ->mutable_parking_info()
          ->set_parking_space_id(parking_space_id);
      return true;
    }
  }
  ADEBUG << "Failed to fill parking ID";
  //std::cout<<"ADEBUG "<< "Failed to fill parking ID"<<std::endl;

  return false;
}


//主流程

//遍历routing_request，因为routing_request请求为一个个的点，
//所以先查看routing_request的点是否在路上，不在路上则找最近的路，
//并且补充信息（不在路上的点则过不去），
//最后调用navigator_ptr_->SearchRoute返回routing响应
bool Routing::Process(const std::shared_ptr<RoutingRequest>& routing_request,
                      RoutingResponse* const routing_response) {
  //CHECK_NOTNULL(routing_response);

  AINFO << "Get new routing request:" << routing_request->DebugString();
  //std::cout << "Get new routing request:" << routing_request->DebugString()<<std::endl;

  //找到routing_request节点最近的路
  //从地图中选择最佳的匹配点
  const auto& fixed_requests = FillLaneInfoIfMissing(*routing_request);
  double min_routing_length = std::numeric_limits<double>::max();
  for (const auto& fixed_request : fixed_requests) 
  {
    RoutingResponse routing_response_temp;
    // 是否能找到规划路径
    if (navigator_ptr_->SearchRoute(fixed_request, &routing_response_temp)) {
      const double routing_length =
          routing_response_temp.measurement().distance();
      if (routing_length < min_routing_length) {
        routing_response->CopyFrom(routing_response_temp);
        min_routing_length = routing_length;
      }
    }
    FillParkingID(routing_response);
  }
  if (min_routing_length < std::numeric_limits<double>::max()) {
    //monitor_logger_buffer_.INFO("Routing success!");
    return true;
  }
  AERROR << "Failed to search route with navigator.";
  //std::cout<<"AERROR "<< "Failed to search route with navigator."<<std::endl;

  //monitor_logger_buffer_.WARN("Routing failed! " +
  //                            routing_response->status().msg());
  return false;
}

}  // namespace routing
}  // namespace apollo
