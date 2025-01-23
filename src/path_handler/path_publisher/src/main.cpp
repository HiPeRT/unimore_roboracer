#include <unistd.h>

#include "path_publisher/path_publisher_node.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  PathPublisherNode ppn;
  sleep(5);
  ppn.publish();
  sleep(5);
}