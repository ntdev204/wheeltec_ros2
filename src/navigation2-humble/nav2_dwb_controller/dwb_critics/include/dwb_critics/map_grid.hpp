


#ifndef DWB_CRITICS__MAP_GRID_HPP_
#define DWB_CRITICS__MAP_GRID_HPP_

#include <vector>
#include <memory>
#include <string>
#include <utility>

#include "dwb_core/trajectory_critic.hpp"
#include "costmap_queue/costmap_queue.hpp"

namespace dwb_critics
{


class MapGridCritic : public dwb_core::TrajectoryCritic
{
public:

  void onInit() override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;
  void addCriticVisualization(
    std::vector<std::pair<std::string, std::vector<float>>> & cost_channels) override;
  double getScale() const override {return costmap_->getResolution() * 0.5 * scale_;}


  

  virtual double scorePose(const geometry_msgs::msg::Pose2D & pose);

  

  inline double getScore(unsigned int x, unsigned int y)
  {
    return cell_values_[costmap_->getIndex(x, y)];
  }

  

  void setAsObstacle(unsigned int index);

protected:
  


  enum class ScoreAggregationType {Last, Sum, Product};

  

  class MapGridQueue : public costmap_queue::CostmapQueue
  {
public:
    MapGridQueue(nav2_costmap_2d::Costmap2D & costmap, MapGridCritic & parent)
    : costmap_queue::CostmapQueue(costmap, true), parent_(parent) {}
    virtual ~MapGridQueue() = default;
    bool validCellToQueue(const costmap_queue::CellData & cell) override;

protected:
    MapGridCritic & parent_;
  };

  

  void reset() override;

  

  void propogateManhattanDistances();

  std::shared_ptr<MapGridQueue> queue_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::vector<double> cell_values_;
  double obstacle_score_, unreachable_score_;
  bool stop_on_failure_;
  ScoreAggregationType aggregationType_;
};
}

#endif
