


#ifndef NAV2_CORE__EXCEPTIONS_HPP_
#define NAV2_CORE__EXCEPTIONS_HPP_

#include <stdexcept>
#include <string>
#include <memory>

namespace nav2_core
{

class PlannerException : public std::runtime_error
{
public:
  explicit PlannerException(const std::string description)
  : std::runtime_error(description) {}
  using Ptr = std::shared_ptr<PlannerException>;
};

}

#endif
