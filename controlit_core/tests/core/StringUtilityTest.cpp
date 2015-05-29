#include <gtest/gtest.h>

#include <controlit/utility/string_utility.hpp>

/* ----------------------------------------------------------------------------
 *
 * --------------------------------------------------------------------------*/
TEST(string_utility, SlowBreakUp)
{
  std::string str("hello.world.nice.to.meet.you");

  controlit::utility::string_splitter splitter(str, '.');

  // Add some registries
  EXPECT_TRUE(splitter.next() == "hello");
  EXPECT_TRUE(splitter.rest() == "world.nice.to.meet.you");
  EXPECT_TRUE(splitter.next() == "world");
  EXPECT_TRUE(splitter.rest() == "nice.to.meet.you");
  EXPECT_TRUE(splitter.next() == "nice");
  EXPECT_TRUE(splitter.rest() == "to.meet.you");
  EXPECT_TRUE(splitter.next() == "to");
  EXPECT_TRUE(splitter.rest() == "meet.you");
  EXPECT_TRUE(splitter.next() == "meet");
  EXPECT_TRUE(splitter.rest() == "you");
  EXPECT_TRUE(splitter.next() == "you");
  EXPECT_TRUE(splitter.rest() == "");
  EXPECT_TRUE(splitter.next() == "");
}
