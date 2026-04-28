

#ifndef DWB_CORE__EXCEPTIONS_HPP_
#define DWB_CORE__EXCEPTIONS_HPP_

#include <stdexcept>
#include <string>
#include <memory>

#include "nav2_core/exceptions.hpp"

namespace dwb_core
{



class PlannerTFException : public nav2_core::PlannerException
{
public:
  explicit PlannerTFException(const std::string description)
  : nav2_core::PlannerException(description) {}
};



class IllegalTrajectoryException : public nav2_core::PlannerException
{
public:
  IllegalTrajectoryException(const std::string critic_name, const std::string description)
  : nav2_core::PlannerException(description), critic_name_(critic_name) {}
  std::string getCriticName() const {return critic_name_;}

protected:
  std::string critic_name_;
};

}

#endif
