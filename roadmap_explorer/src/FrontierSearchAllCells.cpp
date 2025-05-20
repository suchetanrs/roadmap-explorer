#include <roadmap_explorer/costmap_tools.hpp>
#include <roadmap_explorer/FrontierSearchAllCells.hpp>
namespace roadmap_explorer
{

    using nav2_costmap_2d::FREE_SPACE;
    using nav2_costmap_2d::LETHAL_OBSTACLE;
    using nav2_costmap_2d::NO_INFORMATION;

    FrontierSearchAllCells::FrontierSearchAllCells(nav2_costmap_2d::Costmap2D &costmap, int min_frontier_cluster_size, int max_frontier_cluster_size)
        : costmap_(costmap),
          min_frontier_cluster_size_(min_frontier_cluster_size),
          max_frontier_cluster_size_(max_frontier_cluster_size)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("frontier_search_all_cells"), "FrontierSearch::FrontierSearch");
    }

    std::vector<FrontierPtr> FrontierSearchAllCells::searchAllCells()
    {
        //  frontier_list to store the detected frontiers.
        std::vector<FrontierPtr> frontier_list;

        // make sure map is consistent and locked for duration of search
        std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_.getMutex()));

        map_ = costmap_.getCharMap();
        size_x_ = costmap_.getSizeInCellsX();
        size_y_ = costmap_.getSizeInCellsY();

        // initialize flag arrays to keep track of visited and frontier cells
        std::vector<bool> frontier_flag(size_x_ * size_y_, false);

        for (unsigned int mx = 0; mx < size_x_; ++mx)
        {
            for (unsigned int my = 0; my < size_y_; ++my)
            {
                unsigned int idx = costmap_.getIndex(mx, my);

                if (isNewFrontierCell(idx, frontier_flag))
                {
                    // Create a new frontier starting from this cell
                    frontier_flag[idx] = true;
                    std::vector<FrontierPtr> new_frontiers = buildNewFrontier(idx, frontier_flag);

                    for (const auto &frontier : new_frontiers)
                    {
                        if (frontier->getSize() > min_frontier_cluster_size_)
                        {
                            frontier_list.push_back(frontier);
                        }
                    }
                }
            }
        }

        return frontier_list;
    }

    std::vector<FrontierPtr> FrontierSearchAllCells::buildNewFrontier(unsigned int initial_cell, std::vector<bool> &frontier_flag)
    {
        // initialize frontier structure
        FrontierPtr output = std::make_shared<Frontier>();
        // record initial contact point for frontier
        output->setSize(1);
        std::vector<FrontierPtr> calculated_frontiers;
        unsigned int ix, iy;
        costmap_.indexToCells(initial_cell, ix, iy);
        double wx, wy;
        costmap_.mapToWorld(ix, iy, wx, wy);
        output->setGoalPoint(wx, wy);
        output->setUID(generateUID(output));
        calculated_frontiers.push_back(output);

        // push initial gridcell onto queue
        std::queue<unsigned int> bfs;
        bfs.push(initial_cell);
        int current_cluster_size = 0;

        while (!bfs.empty())
        {
            unsigned int idx = bfs.front();
            bfs.pop();

            // try adding cells in 8-connected neighborhood to frontier
            for (unsigned int nbr : nhood8(idx, costmap_))
            {
                // check if neighbour is a potential frontier cell
                if (isNewFrontierCell(nbr, frontier_flag))
                {
                    // mark cell as frontier
                    frontier_flag[nbr] = true;
                    unsigned int mx, my;
                    double wx, wy;
                    costmap_.indexToCells(nbr, mx, my);
                    costmap_.mapToWorld(mx, my, wx, wy);

                    // add to every frontier list
                    std::vector<double> coord_val;
                    coord_val.push_back(wx);
                    coord_val.push_back(wy);

                    every_frontier_list.push_back(coord_val);

                    // update frontier size
                    output->setSize(output->getSize() + 1);

                    // add to queue for breadth first search
                    bfs.push(nbr);

                    if (output->getSize() > max_frontier_cluster_size_)
                    {
                        output->setUID(generateUID(output));
                        output->setGoalPoint(wx, wy);
                        // std::cout << "1PUSHING NEW FRONTIER TO LIST: UID: " << output.unique_id << std::endl;
                        // std::cout << "1Size: " << output.size << std::endl;
                        // std::cout << "1Minimum Distance: " << output.min_distance << std::endl;
                        // std::cout << "1Initial Point: (" << output.goal_point.x << ", " << output.goal_point.y << ", " << output.goal_point.z << ")" << std::endl;
                        // std::cout << "**************" << std::endl;
                        calculated_frontiers.push_back(output);
                        output->setSize(1);
                    }
                }
            }
        }
        return calculated_frontiers;
    }

    bool FrontierSearchAllCells::isNewFrontierCell(unsigned int idx, const std::vector<bool> &frontier_flag)
    {
        // check that cell is unknown and not already marked as frontier
        if (map_[idx] != NO_INFORMATION || frontier_flag[idx])
        {
            return false;
        }

        // frontier cells should have at least one cell in 4-connected neighbourhood that is free
        for (unsigned int nbr : nhood4(idx, costmap_))
        {
            if (map_[nbr] == FREE_SPACE)
            {
                return true;
            }
        }

        return false;
    }

    std::vector<std::vector<double>> FrontierSearchAllCells::getAllFrontiers()
    {
        return every_frontier_list;
    }
}