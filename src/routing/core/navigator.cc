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

/*
#include "modules/routing/core/navigator.h"

#include "cyber/common/file.h"
#include "modules/routing/common/routing_gflags.h"
#include "modules/routing/graph/sub_topo_graph.h"
#include "modules/routing/strategy/a_star_strategy.h"
*/

#include "../../../src/routing/core/navigator.h"

#include "../../../cyber/common/file.h"
#include "../../../src/routing/common/routing_gflags.h"
#include "../../../src/routing/graph/sub_topo_graph.h"
#include "../../../src/routing/strategy/a_star_strategy.h"
namespace apollo {
namespace routing {

namespace {

using apollo::common::ErrorCode;

bool ShowRequestInfo(const RoutingRequest& request, const TopoGraph* graph) {
  for (const auto& wp : request.waypoint()) {
    const auto* node = graph->GetNode(wp.id());
    if (node == nullptr) {
      AERROR << "Way node is not found in topo graph! ID: " << wp.id();
      /*
      std::cout<< "AERROR<< Way node is not found in topo graph! ID: " 
                << wp.id()<<std::endl;
      */
      return false;
    }

    AINFO << "Way point:\tlane id: " << wp.id() << " s: " << wp.s()
          << " x: " << wp.pose().x() << " y: " << wp.pose().y()
          << " length: " << node->Length();
/*
    std::cout<< "Way point:\tlane id: " << wp.id() << " s: " << wp.s()
             << " x: " << wp.pose().x() << " y: " << wp.pose().y()
             << " length: " << node->Length()<<std::endl;
*/
  }

  for (const auto& bl : request.blacklisted_lane()) {
    const auto* node = graph->GetNode(bl.id());
    if (node == nullptr) {
      AERROR << "Black list node is not found in topo graph! ID: " << bl.id();
      /*
      std::cout<<"AERROR << Black list node is not found in topo graph! ID: "
               << bl.id()<<std::endl;
      */
      return false;
    }

    AINFO << "Black point:\tlane id: " << bl.id()
          << " start_s: " << bl.start_s() << " end_s: " << bl.end_s()
          << " length: " << node->Length();
/*
    std::cout << "Black point:\tlane id: " << bl.id()
          << " start_s: " << bl.start_s() << " end_s: " << bl.end_s()
          << " length: " << node->Length()<<std::endl;
*/
  }

  return true;
}

bool GetWayNodes(const RoutingRequest& request, const TopoGraph* graph,
                 std::vector<const TopoNode*>* const way_nodes,
                 std::vector<double>* const way_s) {
  for (const auto& point : request.waypoint()) {
    const auto* cur_node = graph->GetNode(point.id());
    if (cur_node == nullptr) {
      AERROR << "Cannot find way point in graph! Id: " << point.id();
      return false;
    }
    way_nodes->push_back(cur_node);
    way_s->push_back(point.s());
  }
  return true;
}

void SetErrorCode(const common::ErrorCode& error_code_id,
                  const std::string& error_string,
                  common::StatusPb* const error_code) {
  error_code->set_error_code(error_code_id);
  error_code->set_msg(error_string);
  if (error_code_id == common::ErrorCode::OK) {
    ADEBUG << error_string.c_str();
    //std::cout<<"ADEBUG "<< error_string.c_str()<<std::endl;
  } else {
    AERROR << error_string.c_str();
    //std::cout<<"AERROR "<< error_string.c_str()<<std::endl;
  }
}

void PrintDebugData(const std::vector<NodeWithRange>& nodes) {
  AINFO << "Route lane id\tis virtual\tstart s\tend s";
  //std::cout << "Route lane id\tis virtual\tstart s\tend s"<<std::endl;  

  for (const auto& node : nodes) {
    
    AINFO << node.GetTopoNode()->LaneId() << "\t"
          << node.GetTopoNode()->IsVirtual() << "\t" << node.StartS() << "\t"
          << node.EndS();
/*
    std::cout << node.GetTopoNode()->LaneId() << "\t"
          << node.GetTopoNode()->IsVirtual() << "\t" << node.StartS() << "\t"
          << node.EndS() <<std::endl;
*/          
  }
}

}  // namespace

Navigator::Navigator(const std::string& topo_file_path) {
  Graph graph;
  if (!cyber::common::GetProtoFromFile(topo_file_path, &graph)) {
    AERROR << "Failed to read topology graph from " << topo_file_path;
    /*
    std::cout<< "Failed to read topology graph from " 
             << topo_file_path<<std::endl;
    */         
    return;
  }

  graph_.reset(new TopoGraph());
  if (!graph_->LoadGraph(graph)) {

    AINFO << "Failed to init navigator graph failed! File path: "
          << topo_file_path;
/*
    std::cout<< "Failed to init navigator graph failed! File path: "
          << topo_file_path <<std::endl;
*/
    return;
  }
  black_list_generator_.reset(new BlackListRangeGenerator);
  result_generator_.reset(new ResultGenerator);
  is_ready_ = true;
  AINFO << "The navigator is ready.";
  //std::cout << "The navigator is ready."<<std::endl;
}

Navigator::~Navigator() {}

bool Navigator::IsReady() const { return is_ready_; }

void Navigator::Clear() { topo_range_manager_.Clear(); }



//???routing_request?????????graph???topo????????????????????????????????????way_nodes???way_s
//?????????routing_request???????????????????????????topo_range_manager_
bool Navigator::Init(const RoutingRequest& request, const TopoGraph* graph,
                     std::vector<const TopoNode*>* const way_nodes,
                     std::vector<double>* const way_s) {
  Clear();
  //??????routing??????????????????????????????????????????????????????????????????way_nodes???way_s
  if (!GetWayNodes(request, graph_.get(), way_nodes, way_s)) {
    AERROR << "Failed to find search terminal point in graph!";
    return false;
  }
  //????????????????????????????????????lane
  //???routing???????????????????????????????????????????????????routing?????????????????????????????????
  black_list_generator_->GenerateBlackMapFromRequest(request, graph_.get(),
                                                     &topo_range_manager_);
  return true;
}

bool Navigator::MergeRoute(
    const std::vector<NodeWithRange>& node_vec,
    std::vector<NodeWithRange>* const result_node_vec) const {
  for (const auto& node : node_vec) {
    if (result_node_vec->empty() ||
        result_node_vec->back().GetTopoNode() != node.GetTopoNode()) {
      result_node_vec->push_back(node);
    } else {
      if (result_node_vec->back().EndS() < node.StartS()) {
        AERROR << "Result route is not continuous.";
        return false;
      }
      result_node_vec->back().SetEndS(node.EndS());
    }
  }
  return true;
}

//?????????
bool Navigator::SearchRouteByStrategy(
    const TopoGraph* graph, const std::vector<const TopoNode*>& way_nodes,
    const std::vector<double>& way_s,
    std::vector<NodeWithRange>* const result_nodes) const {
  
  std::cout<<"Navigator::SearchRouteByStrategy"<<std::endl;

  std::unique_ptr<Strategy> strategy_ptr;
  //??????AStar
  strategy_ptr.reset(new AStarStrategy(FLAGS_enable_change_lane_in_result));

  result_nodes->clear();
  std::vector<NodeWithRange> node_vec;
  //??????routing_request??????
  for (size_t i = 1; i < way_nodes.size(); ++i) {
    const auto* way_start = way_nodes[i - 1];
    const auto* way_end = way_nodes[i];
    double way_start_s = way_s[i - 1];
    double way_end_s = way_s[i];

    TopoRangeManager full_range_manager = topo_range_manager_;
    //????????????????????????????????????????????????????????????????????????
    black_list_generator_->AddBlackMapFromTerminal(
        way_start, way_end, way_start_s, way_end_s, &full_range_manager);

    //??????????????????????????????????????????????????????????????????????????????2????????????
    //2?????????????????????????????????
    SubTopoGraph sub_graph(full_range_manager.RangeMap());
    //????????????
    const auto* start = sub_graph.GetSubNodeWithS(way_start, way_start_s);
    if (start == nullptr) {

      AERROR << "Sub graph node is nullptr, origin node id: "
             << way_start->LaneId() << ", s:" << way_start_s;
/*
      std::cout << "Sub graph node is nullptr, origin node id: "
             << way_start->LaneId() << ", s:" << way_start_s <<std::endl;
*/            
      return false;
    }
    //????????????
    const auto* end = sub_graph.GetSubNodeWithS(way_end, way_end_s);
    if (end == nullptr) {

      AERROR << "Sub graph node is nullptr, origin node id: "
             << way_end->LaneId() << ", s:" << way_end_s;
/*
      std::cout << "Sub graph node is nullptr, origin node id: "
             << way_end->LaneId() << ", s:" << way_end_s <<std::endl;
*/             
      return false;
    }

    //??????AStar
    std::vector<NodeWithRange> cur_result_nodes;
    if (!strategy_ptr->Search(graph, &sub_graph, start, end,
                              &cur_result_nodes)) {

      AERROR << "Failed to search route with waypoint from " << start->LaneId()
             << " to " << end->LaneId();
/*
      std::cout << "Failed to search route with waypoint from " << start->LaneId()
             << " to " << end->LaneId() <<std::endl;   
*/                      
      return false;
    }

    //???????????????node_vec
    node_vec.insert(node_vec.end(), cur_result_nodes.begin(),
                    cur_result_nodes.end());
  }

  //??????Route
  if (!MergeRoute(node_vec, result_nodes)) {
    AERROR << "Failed to merge route.";
    return false;
  }
  return true;
}


//??????
bool Navigator::SearchRoute(const RoutingRequest& request,
                            RoutingResponse* const response) {
  std::cout<<"Navigator::SearchRoute" <<std::endl;                          

  if (!ShowRequestInfo(request, graph_.get())) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_REQUEST,
                 "Error encountered when reading request point!",
                 response->mutable_status());
    return false;
  }

  if (!IsReady()) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_NOT_READY, "Navigator is not ready!",
                 response->mutable_status());
    return false;
  }
  std::vector<const TopoNode*> way_nodes;
  std::vector<double> way_s;
  if (!Init(request, graph_.get(), &way_nodes, &way_s)) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_NOT_READY,
                 "Failed to initialize navigator!", response->mutable_status());
    return false;
  }

  std::vector<NodeWithRange> result_nodes;
  if (!SearchRouteByStrategy(graph_.get(), way_nodes, way_s, &result_nodes)) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_RESPONSE,
                 "Failed to find route with request!",
                 response->mutable_status());
    return false;
  }
  if (result_nodes.empty()) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_RESPONSE, "Failed to result nodes!",
                 response->mutable_status());
    return false;
  }
  result_nodes.front().SetStartS(request.waypoint().begin()->s());
  result_nodes.back().SetEndS(request.waypoint().rbegin()->s());

  if (!result_generator_->GeneratePassageRegion(
          graph_->MapVersion(), request, result_nodes, topo_range_manager_,
          response)) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_RESPONSE,
                 "Failed to generate passage regions based on result lanes",
                 response->mutable_status());
    return false;
  }
  SetErrorCode(ErrorCode::OK, "Success!", response->mutable_status());

  PrintDebugData(result_nodes);
  return true;
}

}  // namespace routing
}  // namespace apollo
