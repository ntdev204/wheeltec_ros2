


#ifndef DWB_CORE__ILLEGAL_TRAJECTORY_TRACKER_HPP_
#define DWB_CORE__ILLEGAL_TRAJECTORY_TRACKER_HPP_

#include <map>
#include <utility>
#include <string>
#include "dwb_core/exceptions.hpp"
#include "nav2_core/exceptions.hpp"

namespace dwb_core
{
class IllegalTrajectoryTracker
{
public:
  IllegalTrajectoryTracker()
  : legal_count_(0), illegal_count_(0) {}

  void addIllegalTrajectory(const IllegalTrajectoryException & e);
  void addLegalTrajectory();

  std::map<std::pair<std::string, std::string>, double> getPercentages() const;

  std::string getMessage() const;

protected:
  std::map<std::pair<std::string, std::string>, unsigned int> counts_;
  unsigned int legal_count_, illegal_count_;
};



class NoLegalTrajectoriesException
  : public nav2_core::PlannerException
{
public:
  explicit NoLegalTrajectoriesException(const IllegalTrajectoryTracker & tracker)
  : PlannerException(tracker.getMessage()),
    tracker_(tracker) {}
  IllegalTrajectoryTracker tracker_;
};

}

#endif
