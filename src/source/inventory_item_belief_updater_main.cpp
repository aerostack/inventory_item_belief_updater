#include "../include/inventory_item_belief_updater.h"
#include <boost/thread/thread.hpp>


int main(int argc, char** argv){
  ros::init(argc, argv, ros::this_node::getName());
  std::cout << ros::this_node::getName() << std::endl;
  InventoryItemBeliefUpdater detector;
  detector.ownSetUp();
    ros::Rate rate(30);
  ros::spin();
  return 0;
}
