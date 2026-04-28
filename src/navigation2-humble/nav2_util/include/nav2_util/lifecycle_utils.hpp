













#ifndef NAV2_UTIL__LIFECYCLE_UTILS_HPP_
#define NAV2_UTIL__LIFECYCLE_UTILS_HPP_

#include <vector>
#include <string>
#include <chrono>
#include "nav2_util/string_utils.hpp"

namespace nav2_util
{




void startup_lifecycle_nodes(
  const std::vector<std::string> & node_names,
  const std::chrono::seconds service_call_timeout = std::chrono::seconds::max(),
  const int retries = 3);




void startup_lifecycle_nodes(
  const std::string & nodes,
  const std::chrono::seconds service_call_timeout = std::chrono::seconds::max(),
  const int retries = 3)
{
  startup_lifecycle_nodes(split(nodes, ':'), service_call_timeout, retries);
}




void reset_lifecycle_nodes(
  const std::vector<std::string> & node_names,
  const std::chrono::seconds service_call_timeout = std::chrono::seconds::max(),
  const int retries = 3);




void reset_lifecycle_nodes(
  const std::string & nodes,
  const std::chrono::seconds service_call_timeout = std::chrono::seconds::max(),
  const int retries = 3)
{
  reset_lifecycle_nodes(split(nodes, ':'), service_call_timeout, retries);
}

}

#endif
