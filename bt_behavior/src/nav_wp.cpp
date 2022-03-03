// Copyright 2021 Intelligent Robotics Lab
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

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  //Cambiar nombre nodo
  auto node = rclcpp::Node::make_shared("nav_wp");
  int index = 0;

  node->declare_parameter<std::vector<std::string>>("waypoints", {});
  std::vector<std::string> wp_ids;
  node->get_parameter("waypoints", wp_ids);

  for (const auto & wp_id : wp_ids) {
    node->declare_parameter<std::vector<double>>(wp_id, {});
    std::vector<double> wp_coords;
    node->get_parameter(wp_id, wp_coords);
  }

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  //Cambiar qui y en el cmake el nombre del nodo
  factory.registerFromPlugin(loader.getOSName("br2_move_bt_node"));
  factory.registerFromPlugin(loader.getOSName("br2_gnw_bt_node"));
  factory.registerFromPlugin(loader.getOSName("br2_irf_bt_node"));

  std::string pkgpath = ament_index_cpp::get_package_share_directory("bt_behavior");
  std::string xml_file = pkgpath + "/behavior_tree_xml/behavior.xml";

  auto blackboard = BT::Blackboard::create();
  //quizas aqui añadir iterador a 0
  blackboard->set("node", node);
  blackboard->set("index", index);
  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  //Yo diria de quitar esto
  auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 2666, 2667);

  rclcpp::Rate rate(10);

  bool finish = false;
  while (!finish && rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;

    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

// ros2 run pkg node --ros-args --params-file ruta absoluta