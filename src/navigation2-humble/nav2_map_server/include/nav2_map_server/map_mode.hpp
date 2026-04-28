













#ifndef NAV2_MAP_SERVER__MAP_MODE_HPP_
#define NAV2_MAP_SERVER__MAP_MODE_HPP_

#include <string>
#include <vector>
namespace nav2_map_server
{


enum class MapMode
{
  

  Trinary,
  

  Scale,
  

  Raw,
};



const char * map_mode_to_string(MapMode map_mode);



MapMode map_mode_from_string(std::string map_mode_name);
}

#endif
