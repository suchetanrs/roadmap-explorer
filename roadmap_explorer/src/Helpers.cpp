#include "roadmap_explorer/Helpers.hpp"

namespace roadmap_explorer
{
std::vector<unsigned int> nhood4_values(4);
std::vector<unsigned int> nhood8_values(8);

void bresenham2D(
  RayTracedCells & at, unsigned int abs_da, unsigned int abs_db, int error_b,
  int offset_a,
  int offset_b, unsigned int offset,
  unsigned int max_length,
  int resolution_cut_factor)
{
  unsigned int end = std::min(max_length, abs_da);
  for (unsigned int i = 0; i < end; ++i) {
    if (i % resolution_cut_factor == 0) {
      at(offset);
    }
    offset += offset_a;
    error_b += abs_db;
    if ((unsigned int)error_b >= abs_da) {
      offset += offset_b;
      error_b -= abs_da;
    }
  }
  at(offset);
}

bool getTracedCells(
  double sx, double sy, double wx, double wy, RayTracedCells & cell_gatherer, double max_length,
  nav2_costmap_2d::Costmap2D * exploration_costmap_)
{
  unsigned int min_length = 0.0;
  int resolution_cut_factor = 1;
  // Calculate map coordinates
  unsigned int x1, y1;
  unsigned int x0, y0;
  if (!exploration_costmap_->worldToMap(wx, wy, x1, y1) ||
    !exploration_costmap_->worldToMap(sx, sy, x0, y0))
  {
    std::cerr << "Not world to map" << std::endl;
    return false;
  }

  // Calculate distance and adjust starting point to min_length distance
  int dx_full = x1 - x0;
  int dy_full = y1 - y0;
  double dist = std::hypot(dx_full, dy_full);
  if (dist < min_length) {
    LOG_ERROR("Distance to ray trace is lesser than minimum distance. Proceeding to next frontier.")
    return false;
  }
  unsigned int min_x0, min_y0;
  if (dist > 0.0) {
    // Adjust starting point and offset to start from min_length distance
    min_x0 = (unsigned int)(x0 + dx_full / dist * min_length);
    min_y0 = (unsigned int)(y0 + dy_full / dist * min_length);
  } else {
    min_x0 = x0;
    min_y0 = y0;
  }
  unsigned int offset = min_y0 * exploration_costmap_->getSizeInCellsX() + min_x0;

  int dx = x1 - min_x0;
  int dy = y1 - min_y0;

  unsigned int abs_dx = abs(dx);
  unsigned int abs_dy = abs(dy);

  int offset_dx = sign(dx);
  int offset_dy = sign(dy) * exploration_costmap_->getSizeInCellsX();

  double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_length / dist);
  // Calculate the maximum number of steps based on resolution_cut_factor
  // if x is dominant
  if (abs_dx >= abs_dy) {
    int error_y = abs_dx / 2;

    roadmap_explorer::bresenham2D(
      cell_gatherer, abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset,
      (unsigned int)(scale * abs_dx), resolution_cut_factor);
  } else {
    // otherwise y is dominant
    int error_x = abs_dy / 2;
    roadmap_explorer::bresenham2D(
      cell_gatherer, abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset,
      (unsigned int)(scale * abs_dy), resolution_cut_factor);
  }
  return true;
}

bool isCircleFootprintInLethal(
  const nav2_costmap_2d::Costmap2D * costmap, unsigned int center_x,
  unsigned int center_y, double radius_in_cells)
{
  unsigned int size_x_ = costmap->getSizeInCellsX();
  unsigned int size_y_ = costmap->getSizeInCellsY();
  for (int dx = -radius_in_cells; dx <= radius_in_cells; ++dx) {
    for (int dy = -radius_in_cells; dy <= radius_in_cells; ++dy) {
      // Check if the point is within the circle
      if (dx * dx + dy * dy <= radius_in_cells * radius_in_cells) {
        unsigned int x = center_x + dx;
        unsigned int y = center_y + dy;
        if (x >= size_x_ || y >= size_y_) {
          continue;               // Out of bounds
        }
        unsigned int cost = costmap->getCost(x, y);
        if (cost == 254) {
          return true;               // Robot does not fit
        }
      }
    }
  }
  return false;
}

std::vector<unsigned int> nhood4(unsigned int idx, const nav2_costmap_2d::Costmap2D & costmap)
{
  // get 4-connected neighbourhood indexes, check for edge of map
  unsigned int size_x_ = costmap.getSizeInCellsX();
  unsigned int size_y_ = costmap.getSizeInCellsY();

  if (idx > size_x_ * size_y_ - 1) {
    nhood4_values.clear();
    return nhood4_values;
  }

  if (nhood4_values.size() == 0) {
    nhood4_values.resize(4);
  }

  if (idx % size_x_ > 0) {
    nhood4_values[0] = idx - 1;
  }
  if (idx % size_x_ < size_x_ - 1) {
    nhood4_values[1] = idx + 1;
  }
  if (idx >= size_x_) {
    nhood4_values[2] = idx - size_x_;
  }
  if (idx < size_x_ * (size_y_ - 1)) {
    nhood4_values[3] = idx + size_x_;
  }
  return nhood4_values;
}

std::vector<unsigned int> nhood8(unsigned int idx, const nav2_costmap_2d::Costmap2D & costmap)
{
  unsigned int size_x_ = costmap.getSizeInCellsX();
  unsigned int size_y_ = costmap.getSizeInCellsY();

  if (idx > size_x_ * size_y_ - 1) {
    nhood8_values.clear();
    return nhood8_values;
  }

  if (nhood8_values.size() == 0) {
    nhood8_values.resize(4);
  }

  // get 8-connected neighbourhood indexes, check for edge of map
  nhood4(idx, costmap);
  std::copy(nhood4_values.begin(), nhood4_values.end(), nhood8_values.begin());

  if (idx % size_x_ > 0 && idx >= size_x_) {
    nhood8_values[4] = idx - 1 - size_x_;
  }
  if (idx % size_x_ > 0 && idx < size_x_ * (size_y_ - 1)) {
    nhood8_values[5] = idx - 1 + size_x_;
  }
  if (idx % size_x_ < size_x_ - 1 && idx >= size_x_) {
    nhood8_values[6] = idx + 1 - size_x_;
  }
  if (idx % size_x_ < size_x_ - 1 && idx < size_x_ * (size_y_ - 1)) {
    nhood8_values[7] = idx + 1 + size_x_;
  }

  return nhood8_values;
}

std::vector<unsigned int> nhood20(unsigned int idx, const nav2_costmap_2d::Costmap2D & costmap)
{
  // get 8-connected neighbourhood indexes, check for edge of map
  std::vector<unsigned int> out = nhood4(idx, costmap);
  auto out_copy = out;
  for (auto val : out_copy) {
    auto out_4 = nhood8(val, costmap);
    out.insert(out.end(), out_4.begin(), out_4.end());
  }

  return out;
}

bool nearestFreeCell(
  unsigned int & result, unsigned int start, unsigned char val,
  const nav2_costmap_2d::Costmap2D & costmap)
{

  const unsigned char * map = costmap.getCharMap();
  const unsigned int size_x = costmap.getSizeInCellsX();
  const unsigned int size_y = costmap.getSizeInCellsY();

  if (start >= size_x * size_y) {
    return false;
  }

  // initialize breadth first search
  std::queue<unsigned int> bfs;
  std::vector<bool> visited_flag(size_x * size_y, false);

  // push initial cell
  bfs.push(start);
  visited_flag[start] = true;

  // search for neighbouring cell matching value
  while (!bfs.empty() && rclcpp::ok()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // return if cell of correct value is found
    if (map[idx] < val) {
      result = idx;
      return true;
    }

    // iterate over all adjacent unvisited cells
    for (unsigned nbr : nhood8(idx, costmap)) {
      if (!visited_flag[nbr]) {
        bfs.push(nbr);
        visited_flag[nbr] = true;
      }
    }
  }

  return false;
}

}
