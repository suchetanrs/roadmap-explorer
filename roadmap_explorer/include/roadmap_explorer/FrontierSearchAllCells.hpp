#ifndef FRONTIER_SEARCH_ALL_HPP_
#define FRONTIER_SEARCH_ALL_HPP_

#include <list>
#include <vector>
#include <queue>
#include <limits>

#include <rclcpp/rclcpp.hpp>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/cost_values.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <roadmap_explorer/Frontier.hpp>

namespace roadmap_explorer
{
     
    class FrontierSearchAllCells
    {
    public:
        FrontierSearchAllCells(nav2_costmap_2d::Costmap2D &costmap, int min_frontier_cluster_size, int max_frontier_cluster_size);

        std::vector<FrontierPtr> searchAllCells();

        std::vector<std::vector<double>> getAllFrontiers();

    private:
        nav2_costmap_2d::Costmap2D &costmap_;
        unsigned char *map_;
        unsigned int size_x_, size_y_;
        std::vector<std::vector<double>> every_frontier_list;
        int min_frontier_cluster_size_;
        int max_frontier_cluster_size_;

        bool isNewFrontierCell(unsigned int idx, const std::vector<bool> &frontier_flag);

        std::vector<FrontierPtr> buildNewFrontier(unsigned int initial_cell, std::vector<bool> &frontier_flag);
    };
}
#endif