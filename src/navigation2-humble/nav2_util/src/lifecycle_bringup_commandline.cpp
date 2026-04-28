













#include <vector>
#include <string>
#include <iostream>
#include <cstdlib>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_utils.hpp"

using std::cerr;
using namespace std::chrono_literals;

void usage()
{
  cerr << "Invalid command line.\n\n";
  cerr << "This command will take a set of unconfigured lifecycle nodes through the\n";
  cerr << "CONFIGURED to the ACTIVATED state\n";
  cerr << "The nodes are brought up in the order listed on the command line\n\n";
  cerr << "Usage:\n";
  cerr << " > lifecycle_startup <node name> ...\n";
  std::exit(1);
}

int main(int argc, char * argv[])
{
  if (argc == 1) {
    usage();
  }
  rclcpp::init(0, nullptr);
  nav2_util::startup_lifecycle_nodes(
    std::vector<std::string>(argv + 1, argv + argc),
    10s);
  rclcpp::shutdown();
}
