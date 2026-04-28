













#ifndef NAV2_UTIL__CLEAR_ENTIRELY_COSTMAP_SERVICE_CLIENT_HPP_
#define NAV2_UTIL__CLEAR_ENTIRELY_COSTMAP_SERVICE_CLIENT_HPP_

#include <string>
#include "nav2_util/service_client.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"

namespace nav2_util
{


class ClearEntirelyCostmapServiceClient
  : public nav2_util::ServiceClient<nav2_msgs::srv::ClearEntireCostmap>
{
public:
  

  explicit ClearEntirelyCostmapServiceClient(const std::string & service_name)
  : nav2_util::ServiceClient<nav2_msgs::srv::ClearEntireCostmap>(service_name)
  {
  }

  using clearEntirelyCostmapServiceRequest =
    nav2_util::ServiceClient<nav2_msgs::srv::ClearEntireCostmap>::RequestType;
  using clearEntirelyCostmapServiceResponse =
    nav2_util::ServiceClient<nav2_msgs::srv::ClearEntireCostmap>::ResponseType;
};

}

#endif
