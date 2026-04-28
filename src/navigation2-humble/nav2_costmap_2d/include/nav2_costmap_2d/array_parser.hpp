

#ifndef NAV2_COSTMAP_2D__ARRAY_PARSER_HPP_
#define NAV2_COSTMAP_2D__ARRAY_PARSER_HPP_

#include <vector>
#include <string>

namespace nav2_costmap_2d
{



std::vector<std::vector<float>> parseVVF(const std::string & input, std::string & error_return);

}

#endif
