














#ifndef NAV2_MPPI_CONTROLLER__CRITICS__PATH_FOLLOW_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__PATH_FOLLOW_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{



class PathFollowCritic : public CriticFunction
{
public:
  

  void initialize() override;

  

  void score(CriticData & data) override;

protected:
  float threshold_to_consider_{0};
  size_t offset_from_furthest_{0};

  unsigned int power_{0};
  float weight_{0};
};

}

#endif
