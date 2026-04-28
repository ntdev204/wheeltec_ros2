













#ifndef NAV2_MPPI_CONTROLLER__CRITICS__TWIRLING_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__TWIRLING_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{



class TwirlingCritic : public CriticFunction
{
public:
  

  void initialize() override;

  

  void score(CriticData & data) override;

protected:
  unsigned int power_{0};
  float weight_{0};
};

}

#endif
