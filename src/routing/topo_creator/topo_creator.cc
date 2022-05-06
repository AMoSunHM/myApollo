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
#include "cyber/common/file.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "../../../modules/routing/common/routing_gflags.h"
#include "modules/routing/topo_creator/graph_creator.h"
*/
#include "../../../cyber/common/file.h"
#include "../../../src/map/hdmap/hdmap_util.h"
#include "../../../src/routing/common/routing_gflags.h"
#include "../../../src/routing/topo_creator/graph_creator.h"

#include <iostream>

int main(int argc, char **argv) {
  //google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  apollo::routing::RoutingConfig routing_conf;

/*
  ACHECK(apollo::cyber::common::GetProtoFromFile(FLAGS_routing_conf_file,
                                                 &routing_conf))
      << "Unable to load routing conf file: " + FLAGS_routing_conf_file;
*/
  if(!apollo::cyber::common::GetProtoFromFile(FLAGS_routing_conf_file,
                                                 &routing_conf))
  {
    std::cout<< "Unable to load routing conf file: " + FLAGS_routing_conf_file 
                <<std::endl;
    return 0;
  };

  AINFO << "Conf file: " << FLAGS_routing_conf_file << " is loaded.";
  //std::cout << "Conf file: " << FLAGS_routing_conf_file << " is loaded." <<std::endl;
  std::cout << " base_speed" <<routing_conf.base_speed() <<std::endl;

  const auto base_map = apollo::hdmap::BaseMapFile();
  const auto routing_map = apollo::hdmap::RoutingMapFile();

/*
  //const auto base_map = "/home/casicapollo/Documents/myAll/my_3_3_1/src/map/data/demo/base_map.txt";
  const auto base_map = "/home/casicapollo/Documents/myAll/my_3_3_1/src/map/data/base_map.xml";
  //const auto routing_map = "/home/casicapollo/Documents/myAll/my_3_3_1/src/map/data/demo/routing_map.txt";
  const auto routing_map = "/home/casicapollo/Documents/myAll/my_3_3_1/src/map/data/routing_map.txt";
*/
  apollo::routing::GraphCreator creator(base_map, routing_map, routing_conf);
  ACHECK(creator.Create()) << "Create routing topo failed!";
/*
  if(!creator.Create()) 
  {
    std::cout<<"Create routing topo failed!"<<std::endl;
    return 0;
  }
*/

  AINFO << "Create routing topo successfully from " << base_map << " to "
        << routing_map;
/*
  std::cout << "*****Create routing topo successfully from " << base_map << " to "
        << routing_map<<std::endl;  
*/
  return 0;
}
