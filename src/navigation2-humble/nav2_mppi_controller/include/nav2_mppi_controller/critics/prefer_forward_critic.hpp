













#ifndef NAV2_MPPI_CONTROLLER__CRITICS__PREFER_FORWARD_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__PREFER_FORWARD_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{



class PreferForwardCritic : public CriticFunction
{
public:
  

  void initialize() override;

  

  void score(CriticData & data) override;

protected:
  unsigned int power_{0};
  float weight_{0};
  float threshold_to_consider_{0};
};

}

#endif
