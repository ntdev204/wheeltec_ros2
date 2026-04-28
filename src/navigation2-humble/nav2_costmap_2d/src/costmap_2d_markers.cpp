

#include <string>
#include <vector>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav2_msgs/msg/voxel_grid.hpp"
#include "nav2_voxel_grid/voxel_grid.hpp"
#include "nav2_util/execution_timer.hpp"

struct Cell
{
  double x;
  double y;
  double z;
  nav2_voxel_grid::VoxelStatus status;
};
typedef std::vector<Cell> V_Cell;

float g_colors_r[] = {0.0f, 0.0f, 1.0f};
float g_colors_g[] = {0.0f, 0.0f, 0.0f};
float g_colors_b[] = {0.0f, 1.0f, 0.0f};
float g_colors_a[] = {0.0f, 0.5f, 1.0f};

V_Cell g_cells;
rclcpp::Node::SharedPtr g_node;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub;

void voxelCallback(const nav2_msgs::msg::VoxelGrid::ConstSharedPtr grid)
{
  if (grid->data.empty()) {
    RCLCPP_ERROR(g_node->get_logger(), "Received voxel grid");
    return;
  }

  nav2_util::ExecutionTimer timer;
  timer.start();

  RCLCPP_DEBUG(g_node->get_logger(), "Received voxel grid");

  const std::string frame_id = grid->header.frame_id;
  const rclcpp::Time stamp = grid->header.stamp;
  const uint32_t * data = &grid->data.front();
  const double x_origin = grid->origin.x;
  const double y_origin = grid->origin.y;
  const double z_origin = grid->origin.z;
  const double x_res = grid->resolutions.x;
  const double y_res = grid->resolutions.y;
  const double z_res = grid->resolutions.z;
  const uint32_t x_size = grid->size_x;
  const uint32_t y_size = grid->size_y;
  const uint32_t z_size = grid->size_z;

  g_cells.clear();
  uint32_t num_markers = 0;
  for (uint32_t y_grid = 0; y_grid < y_size; ++y_grid) {
    for (uint32_t x_grid = 0; x_grid < x_size; ++x_grid) {
      for (uint32_t z_grid = 0; z_grid < z_size; ++z_grid) {
        nav2_voxel_grid::VoxelStatus status =
          nav2_voxel_grid::VoxelGrid::getVoxel(
          x_grid, y_grid,
          z_grid, x_size, y_size, z_size, data);
        if (status == nav2_voxel_grid::MARKED) {
          Cell c;
          c.status = status;
          c.x = x_origin + (x_grid + 0.5) * x_res;
          c.y = y_origin + (y_grid + 0.5) * y_res;
          c.z = z_origin + (z_grid + 0.5) * z_res;
          g_cells.push_back(c);

          ++num_markers;
        }
      }
    }
  }

  auto m = std::make_unique<visualization_msgs::msg::Marker>();
  m->header.frame_id = frame_id;
  m->header.stamp = stamp;
  m->ns = g_node->get_namespace();
  m->id = 0;
  m->type = visualization_msgs::msg::Marker::CUBE_LIST;
  m->action = visualization_msgs::msg::Marker::ADD;
  m->pose.orientation.w = 1.0;
  m->scale.x = x_res;
  m->scale.y = y_res;
  m->scale.z = z_res;
  m->color.r = g_colors_r[nav2_voxel_grid::MARKED];
  m->color.g = g_colors_g[nav2_voxel_grid::MARKED];
  m->color.b = g_colors_b[nav2_voxel_grid::MARKED];
  m->color.a = g_colors_a[nav2_voxel_grid::MARKED];
  m->points.resize(num_markers);
  for (uint32_t i = 0; i < num_markers; ++i) {
    Cell & c = g_cells[i];
    geometry_msgs::msg::Point & p = m->points[i];
    p.x = c.x;
    p.y = c.y;
    p.z = c.z;
  }

  pub->publish(std::move(m));

  timer.end();
  RCLCPP_INFO(
    g_node->get_logger(), "Published %d markers in %f seconds",
    num_markers, timer.elapsed_time_in_seconds());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("costmap_2d_marker");

  RCLCPP_DEBUG(g_node->get_logger(), "Starting costmap_2d_marker");

  pub = g_node->create_publisher<visualization_msgs::msg::Marker>(
    "visualization_marker", 1);

  auto sub = g_node->create_subscription<nav2_msgs::msg::VoxelGrid>(
    "voxel_grid", rclcpp::SystemDefaultsQoS(), voxelCallback);

  rclcpp::spin(g_node->get_node_base_interface());
}
