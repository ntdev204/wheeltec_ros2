













#ifndef NAV2_UTIL__STRING_UTILS_HPP_
#define NAV2_UTIL__STRING_UTILS_HPP_

#include <string>
#include <vector>

namespace nav2_util
{

typedef std::vector<std::string> Tokens;



std::string strip_leading_slash(const std::string & in);




Tokens split(const std::string & tokenstring, char delimiter);

}

#endif
