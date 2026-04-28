













#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_util/lifecycle_service_client.hpp"

using std::string;
using lifecycle_msgs::msg::Transition;

namespace nav2_util
{

#define RETRY(fn, retries) \
  { \
    int count = 0; \
    while (true) { \
      try { \
        fn; \
        break; \
      } catch (std::runtime_error & e) { \
        ++count; \
        if (count > (retries)) { \
          throw e;} \
      } \
    } \
  }

static void startupLifecycleNode(
  const std::string & node_name,
  const std::chrono::seconds service_call_timeout,
  const int retries)
{
  LifecycleServiceClient sc(node_name);




  RETRY(
    sc.change_state(Transition::TRANSITION_CONFIGURE, service_call_timeout),
    retries);
  RETRY(
    sc.change_state(Transition::TRANSITION_ACTIVATE, service_call_timeout),
    retries);
}

void startup_lifecycle_nodes(
  const std::vector<std::string> & node_names,
  const std::chrono::seconds service_call_timeout,
  const int retries)
{
  for (const auto & node_name : node_names) {
    startupLifecycleNode(node_name, service_call_timeout, retries);
  }
}

static void resetLifecycleNode(
  const std::string & node_name,
  const std::chrono::seconds service_call_timeout,
  const int retries)
{
  LifecycleServiceClient sc(node_name);




  RETRY(
    sc.change_state(Transition::TRANSITION_DEACTIVATE, service_call_timeout),
    retries);
  RETRY(
    sc.change_state(Transition::TRANSITION_CLEANUP, service_call_timeout),
    retries);
}

void reset_lifecycle_nodes(
  const std::vector<std::string> & node_names,
  const std::chrono::seconds service_call_timeout,
  const int retries)
{
  for (const auto & node_name : node_names) {
    resetLifecycleNode(node_name, service_call_timeout, retries);
  }
}

}
