#ifndef MISC_H
#define MISC_H

namespace pure_pursuit_local_planner
{

/**
 * @brief Calculate Euclidean distance between two 2D point datatypes
 * @param point1 object containing fields x and y
 * @param point2 object containing fields x and y
 * @return Euclidean distance: ||point2-point1||
*/
template <typename P1, typename P2>
inline double distance_points2d(const P1& point1, const P2& point2)
{
  return std::sqrt( std::pow(point2.x-point1.x,2) + std::pow(point2.y-point1.y,2) );
}

}
#endif /* MISC_H */