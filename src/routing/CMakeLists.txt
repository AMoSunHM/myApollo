cmake_minimum_required(VERSION 3.0.2)
project(routing)



#abseil
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


##ROS
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs

  routing_msgs
)
##ROS
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES routing
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
   CATKIN_DEPENDS
   routing_msgs
)


find_package(Protobuf REQUIRED)
#find_package(proj REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(gflags REQUIRED)

find_package(absl REQUIRED)


if(Eigen3_FOUND)
    message("eigen3 found")
else(Eigen3_FOUND)
    message("eigen3 not found")
endif(Eigen3_FOUND)


if(Protobuf_FOUND)
    message("Protobuf found")
else(Protobuf_FOUND)
    message("Protobuf not found")
endif(Protobuf_FOUND)

if(gflags_FOUND)
    message("gflags found")
else(gflags_FOUND)
    message("gflags not found")
endif(gflags_FOUND)


message(${PROTOBUF_LIBRARIES})
message(${gflags_LIBRARIES})
#message(${Eigen3_LIBRARIES})
#message(${proj_LIBRARIES})
#message(${tinyxml_LIBRARIES})
#message(${ABSL_LIBRARIES})

#add_subdirectory(abseil-cpp)

#file(GLOB ALL_PROTO_SRC proto/*.proto)
# Generate the .h and .cxx files
#PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${ALL_PROTO_SRC})





###########
## Build ##
###########

##ROS
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

##########CYBER_src###

set(CYBER_COMMON_SRC
    ../../cyber/common/file.cc
)

set(CYBER_SRC
    ../../cyber/binary.cc
    ${CYBER_COMMON_SRC})

############proto
set(PROTO
    #common
    ../proto/protocpp/vehicle_config.pb.cc 
    ../proto/protocpp/direction.pb.cc
    ../proto/protocpp/drive_event.pb.cc
    ../proto/protocpp/drive_state.pb.cc
    ../proto/protocpp/error_code.pb.cc
    ../proto/protocpp/geometry.pb.cc
    ../proto/protocpp/header.pb.cc
    ../proto/protocpp/pnc_point.pb.cc
    ../proto/protocpp/vehicle_signal.pb.cc 

    #localization
    ../proto/protocpp/pose.pb.cc
    ../proto/protocpp/localization.pb.cc
    ../proto/protocpp/localization_status.pb.cc    

    #perception
    ../proto/protocpp/perception_obstacle.pb.cc

    #map
    ../proto/protocpp/map_clear_area.pb.cc
    ../proto/protocpp/map_crosswalk.pb.cc
    ../proto/protocpp/map_geometry.pb.cc
    ../proto/protocpp/map_id.pb.cc
    ../proto/protocpp/map_junction.pb.cc
    ../proto/protocpp/map_lane.pb.cc
    ../proto/protocpp/map_overlap.pb.cc
    ../proto/protocpp/map_parking_space.pb.cc
    ../proto/protocpp/map_pnc_junction.pb.cc
    ../proto/protocpp/map_road.pb.cc
    ../proto/protocpp/map_rsu.pb.cc
    ../proto/protocpp/map_signal.pb.cc
    ../proto/protocpp/map_speed_bump.pb.cc
    ../proto/protocpp/map_speed_control.pb.cc
    ../proto/protocpp/map_stop_sign.pb.cc
    ../proto/protocpp/map_yield_sign.pb.cc
    ../proto/protocpp/map.pb.cc

    ../proto/protocpp/navigation.pb.cc
    ../proto/protocpp/navigator_config.pb.cc
    ../proto/protocpp/relative_map_config.pb.cc

    #routing
    ../proto/protocpp/default_routing.pb.cc
    ../proto/protocpp/poi.pb.cc
    ../proto/protocpp/routing_config.pb.cc
    ../proto/protocpp/routing.pb.cc
    ../proto/protocpp/topo_graph.pb.cc
    )



##########common_src###
#set(COMMON_CONFIGS_PROTO_SRC
    #common/configs/proto/protocpp/error_code.pb.cc
    #common/configs/proto/protocpp/geometry.pb.cc
    #common/configs/proto/protocpp/header.pb.cc
#    proto/vehicle_config.pb.cc 
#    )

set(COMMON_CONFIGS_SRC
    ../common/configs/config_gflags.cc
    ../common/configs/vehicle_config_helper.cc)

set(COMMON_MATH_SRC
    ../common/math/aabox2d.cc
    ../common/math/box2d.cc
    ../common/math/cartesian_frenet_conversion.cc
    ../common/math/line_segment2d.cc
    ../common/math/linear_interpolation.cc
    ../common/math/math_utils.cc
    ../common/math/path_matcher.cc
    ../common/math/polygon2d.cc
    ../common/math/vec2d.cc
)

#set(COMMON_PROTO_SRC
#    proto/direction.pb.cc
#    proto/drive_event.pb.cc
#    proto/drive_state.pb.cc
#    proto/error_code.pb.cc
#    proto/geometry.pb.cc
#    proto/header.pb.cc
#    proto/pnc_point.pb.cc
    #common/proto/protocpp/pose.pb.cc
#    proto/vehicle_signal.pb.cc   
#    )

set(COMMON_UTIL_SRC
    ../common/util/string_util.cc
    ../common/util/util.cc
    )

set(COMMON_SRC
    ${COMMON_CONFIGS_SRC}
    ${COMMON_MATH_SRC}
#    ${COMMON_PROTO_SRC}
    ${COMMON_UTIL_SRC}
    )

############map_src
set(MAP_HDMAP_ADAPTER_XML_PARSER_SRC
    ../map/hdmap/adapter/xml_parser/header_xml_parser.cc
    ../map/hdmap/adapter/xml_parser/junctions_xml_parser.cc
    ../map/hdmap/adapter/xml_parser/lanes_xml_parser.cc
    ../map/hdmap/adapter/xml_parser/objects_xml_parser.cc
    ../map/hdmap/adapter/xml_parser/roads_xml_parser.cc
    ../map/hdmap/adapter/xml_parser/signals_xml_parser.cc
    ../map/hdmap/adapter/xml_parser/util_xml_parser.cc
)

set(MAP_HDMAP_ADAPTER_SRC
    ../map/hdmap/adapter/coordinate_convert_tool.cc
    ../map/hdmap/adapter/opendrive_adapter.cc
    ../map/hdmap/adapter/proto_organizer.cc
    ${MAP_HDMAP_ADAPTER_XML_PARSER_SRC}
    )

set(MAP_HDMAP_SRC
    ../map/hdmap/hdmap_util.cc
    ../map/hdmap/hdmap.cc
    ../map/hdmap/hdmap_common.cc
    ../map/hdmap/hdmap_impl.cc
    ${MAP_HDMAP_ADAPTER_SRC})


#set(MAP_PROTO_SRC
    #map/proto/protocpp/geometry.pb.cc
#    proto/map_clear_area.pb.cc
#    proto/map_crosswalk.pb.cc
#    proto/map_geometry.pb.cc
#    proto/map_id.pb.cc
#    proto/map_junction.pb.cc
#    proto/map_lane.pb.cc
#    proto/map_overlap.pb.cc
#    proto/map_parking_space.pb.cc
#    proto/map_pnc_junction.pb.cc
#   proto/map_road.pb.cc
#    proto/map_rsu.pb.cc
#    proto/map_signal.pb.cc
#    proto/map_speed_bump.pb.cc
#    proto/map_speed_control.pb.cc
#    proto/map_stop_sign.pb.cc
#    proto/map_yield_sign.pb.cc
#    proto/map.pb.cc
#)

set(MAP_SRC
    ${MAP_HDMAP_SRC}
#    ${MAP_PROTO_SRC}
    )


##routing_src

set(ROUTING_COMMON_SRC
    common/routing_gflags.cc)

set(ROUTING_CORE_SRC
    core/black_list_range_generator.cc
    core/navigator.cc
    core/result_generator.cc
    )    

set(ROUTING_GRAPH_SRC
    graph/node_with_range.cc
    graph/sub_topo_graph.cc
    graph/topo_graph.cc
    graph/topo_node.cc
    graph/topo_range_manager.cc
    graph/topo_range.cc
    )

set(ROUTING_STRATEGY_SRC
    strategy/a_star_strategy.cc)

set(TOPO_CREATOR_SRC
    topo_creator/edge_creator.cc
    topo_creator/graph_creator.cc
    topo_creator/node_creator.cc
    topo_creator/topo_creator.cc
)


set(ROUGING_TOPO_CREATOR_SRC
    ${ROUTING_COMMON_SRC}
    ${TOPO_CREATOR_SRC}    
    )

set(ROUTING_SRC
    ${ROUTING_COMMON_SRC}
    ${ROUTING_CORE_SRC}
    ${ROUTING_GRAPH_SRC}
    ${ROUTING_STRATEGY_SRC}  
    src/routing.cc  
    src/routing_ros.cc
    src/routing_node.cc
    )


#####all_src_topo_creator#####
set(ALL_SRC_TOPO_CREATOR
    ${COMMON_SRC}
    ${MAP_SRC}
    ${CYBER_SRC}
    ${ROUGING_TOPO_CREATOR_SRC}
    )


###all_src_routing
set(ALL_SRC_ROUTING
    ${COMMON_SRC}
    ${MAP_SRC}
    ${CYBER_SRC}
    ${ROUTING_SRC}
    )


###########topo_creator############
add_executable(topo_creator
                ${ALL_SRC_TOPO_CREATOR}
                #${PROTO_SRCS}
                #${PROTO_HDRS} 
                ${PROTO}
              )

target_include_directories(topo_creator
    PUBLIC
    ${PROJECT_SOURCE_DIR}

    ${PROTOBUF_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}

    ../cyber
)

target_link_libraries(topo_creator

    absl::base
    absl::synchronization
    absl::strings 

    /usr/lib/x86_64-linux-gnu/libproj.a
#    ${proj_LIBRARIES}    

    /usr/lib/x86_64-linux-gnu/libtinyxml2.so

    ${PROTOBUF_LIBRARIES}

    ${GFLAGS_LIBRARIES}
#    gflags
#    gflags::gflags
#    /usr/local/lib/libgflags.a

    #Eigen3
)

#############routing##############

add_executable(routing
                ${ALL_SRC_ROUTING}
                #${PROTO_SRCS}
                #${PROTO_HDRS} 
                ${PROTO}
             )

target_include_directories(routing
    PUBLIC
    ${PROJECT_SOURCE_DIR}

    ${PROTOBUF_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}

    #../cyber
)

target_link_libraries(routing

    absl::base
    absl::synchronization
    absl::strings 

    /usr/lib/x86_64-linux-gnu/libproj.a
   #${proj_LIBRARIES}    

    /usr/lib/x86_64-linux-gnu/libtinyxml2.so

    ${PROTOBUF_LIBRARIES}

    ${GFLAGS_LIBRARIES}
    #gflags
    #gflags::gflags
    #/usr/local/lib/libgflags.a

    #Eigen3
    #ROS
    ${catkin_LIBRARIES}
)