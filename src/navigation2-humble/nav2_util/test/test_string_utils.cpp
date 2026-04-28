













#include <string>

#include "nav2_util/string_utils.hpp"
#include "gtest/gtest.h"

using nav2_util::split;
using nav2_util::Tokens;

TEST(Split, SplitFunction)
{
  ASSERT_EQ(split("", ':'), Tokens({""}));
  ASSERT_EQ(split("foo", ':'), Tokens{"foo"});
  ASSERT_EQ(split("foo:bar", ':'), Tokens({"foo", "bar"}));
  ASSERT_EQ(split("foo:bar:", ':'), Tokens({"foo", "bar", ""}));
  ASSERT_EQ(split(":", ':'), Tokens({"", ""}));
  ASSERT_EQ(split("foo::bar", ':'), Tokens({"foo", "", "bar"}));
  ASSERT_TRUE(nav2_util::strip_leading_slash(std::string("/hi")) == std::string("hi"));
}
